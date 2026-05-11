package ysyx

import chisel3._
import chisel3.util._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class SPIIO(val ssWidth: Int = 8) extends Bundle {
  val sck = Output(Bool())
  val ss = Output(UInt(ssWidth.W))
  val mosi = Output(Bool())
  val miso = Input(Bool())
}

class spi_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Reset())
    val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val spi = new SPIIO
    val spi_irq_out = Output(Bool())
  })
}

class flash extends BlackBox {
  val io = IO(Flipped(new SPIIO(1)))
}

class APBSPI(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
  val node = APBSlaveNode(Seq(APBSlavePortParameters(
    Seq(APBSlaveParameters(
      address       = address,
      executable    = true,
      supportsRead  = true,
      supportsWrite = true)),
    beatBytes  = 4)))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, _) = node.in(0)
    val spi_bundle = IO(new SPIIO)

    val controllerAddrSet = address(0)
    val flashAddrSet = address(1)

    assert(address.size == 2, "APBSPI module expects two address sets: controller and flash.")
    val device_controller :: device_flash :: Nil = Enum(address.size)

    val controllerHandler = Module(new APBSPI.ControllerHandler)
    val flashHandler = Module(new APBSPI.FlashHandler)

    /* 
    注: 由于引入硬件层面的 XIP 机制,
    可能会造成硬件层面对 flash 的访存与软件指令层面对 flash 访存之间产生冲突.
    (即任意两个硬件设备针对 SPI 接口的访问会发生冲突.)
    为规避冲突, 引入 SPI 接口判忙机制: 当 SCK 信号工作时认为 SPI 接口正忙.
    当 SPI 接口忙而上层 APB 信号又发来新事务时, 先将 APB 事务的处理挂起.
    等 SPI 接口完成事务转为空闲后, 再处理到来的 APB 事务.
     */
    val spiBusyTimer = RegInit(7.U(3.W))
    val spiBusy = Wire(Bool())
    when(spi_bundle.sck) {
      spiBusyTimer := 7.U
    }.otherwise {
      spiBusyTimer := Mux(spiBusyTimer === 0.U, 0.U, spiBusyTimer - 1.U)
    }
    spiBusy := spiBusyTimer =/= 0.U

    val curDeviceComing = Wire(UInt(device_controller.getWidth.W))
    val curDevice = RegInit(device_controller)
    val curDeviceIn = RegInit(APBSPI.SPIInBundle.default(in.params))
    val penableTimer = RegInit(0.U(2.W))
    val penable = Wire(Bool())
    val newAPBReqComing = Wire(Bool())
    assert(
      (() => {
        val psel = Wire(Bool())
        val canDetermineTarget = Wire(Bool())
        psel := in.psel
        canDetermineTarget := Seq(controllerAddrSet, flashAddrSet).map(_.contains(in.paddr)).reduce(_ || _)

        !(psel && !canDetermineTarget)
      })(),
      "APBSPI: psel is set but cannot determine target device from paddr."
    )
    curDeviceComing := MuxCase(device_controller, Seq(
      controllerAddrSet.contains(in.paddr) -> device_controller,
      flashAddrSet.contains(in.paddr) -> device_flash
    ))
    when(in.psel && !spiBusy) {
      curDevice := curDeviceComing
      curDeviceIn.connect(in)
    }
    when(in.pready) {
      penableTimer := 0.U
    }.elsewhen(penableTimer =/= 0.U) {
      penableTimer := penableTimer - 1.U
    }.elsewhen(in.psel && in.penable && !spiBusy) {
      penableTimer := 3.U
    }
    penable := penableTimer =/= 0.U
    newAPBReqComing := curDevice =/= curDeviceComing

    controllerHandler.io.in.psel := curDevice === device_controller
    flashHandler.io.in.psel := curDevice === device_flash
    Seq(controllerHandler, flashHandler).map(_.io).foreach(io => {
      io.in.penable := Mux(in.pready, false.B, penable)
      io.in.pwrite := curDeviceIn.pwrite
      io.in.paddr := curDeviceIn.paddr
      io.in.pprot := curDeviceIn.pprot
      io.in.pwdata := curDeviceIn.pwdata
      io.in.pstrb := curDeviceIn.pstrb
      io.spi_bundle.miso := spi_bundle.miso
    })
    assert(
      Seq(device_controller, device_flash).map(curDevice === _).reduce(_ || _),
      "APBSPI: curDevice is invalid."
    )
    in.pready := Mux(newAPBReqComing, false.B, MuxCase(false.B, Seq(
      (curDevice === device_controller) -> controllerHandler.io.in.pready,
      (curDevice === device_flash) -> flashHandler.io.in.pready
    )))
    in.pslverr := Mux(newAPBReqComing, false.B, MuxCase(false.B, Seq(
      (curDevice === device_controller) -> controllerHandler.io.in.pslverr,
      (curDevice === device_flash) -> flashHandler.io.in.pslverr
    )))
    in.prdata := Mux(newAPBReqComing, 0.U, MuxCase(0.U, Seq(
      (curDevice === device_controller) -> controllerHandler.io.in.prdata,
      (curDevice === device_flash) -> flashHandler.io.in.prdata
    )))
    spi_bundle.sck := MuxCase(false.B, Seq(
      (curDevice === device_controller) -> controllerHandler.io.spi_bundle.sck,
      (curDevice === device_flash) -> flashHandler.io.spi_bundle.sck
    ))
    spi_bundle.ss := MuxCase(0.U, Seq(
      (curDevice === device_controller) -> controllerHandler.io.spi_bundle.ss,
      (curDevice === device_flash) -> flashHandler.io.spi_bundle.ss
    ))
    spi_bundle.mosi := MuxCase(false.B, Seq(
      (curDevice === device_controller) -> controllerHandler.io.spi_bundle.mosi,
      (curDevice === device_flash) -> flashHandler.io.spi_bundle.mosi
    ))
  }
}

object APBSPI {
  class SPIInBundle(val params: APBBundleParameters) extends Bundle {
    val pwrite = Bool()
    val paddr = UInt(params.addrBits.W)
    val pprot = UInt(params.protBits.W)
    val pwdata = UInt(params.dataBits.W)
    val pstrb = UInt((params.dataBits / 8).W)

    def connect(in: APBBundle): Unit = {
      pwrite := in.pwrite
      paddr := in.paddr
      pprot := in.pprot
      pwdata := in.pwdata
      pstrb := in.pstrb
    }
  }

  object SPIInBundle {
    def default(params: APBBundleParameters): SPIInBundle = {
      val bundle = Wire(new SPIInBundle(params))

      bundle.pwrite := false.B
      bundle.paddr := 0.U
      bundle.pprot := 0.U
      bundle.pwdata := 0.U
      bundle.pstrb := 0.U

      bundle
    }
  }

  class HandlerIOBundle extends Bundle {
    val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val spi_bundle = new SPIIO
  }

  object HandlerIOBundle {
    def defaultForMaster(io: HandlerIOBundle): Unit = {
      io.in.psel := false.B
      io.in.penable := false.B
      io.in.pwrite := false.B
      io.in.paddr := 0.U
      io.in.pprot := 0.U
      io.in.pwdata := 0.U
      io.in.pstrb := 0.U
      io.spi_bundle.miso := false.B
    }
  }

  class Handler(val deviceName: String, val addrRange: Seq[AddressSet]) extends Module {
    val io = IO(new HandlerIOBundle)

    private val addrValid = Wire(Bool())
    private val addr = Wire(UInt(32.W))
    when(io.in.psel) {
      addrValid := addrRange.map(_.contains(io.in.paddr)).reduce(_ || _)
      addr := io.in.paddr
    }.otherwise {
      addrValid := true.B
      addr := 0.U
    }
    assert(
      !(io.in.psel && io.in.penable && !addrValid),
      cf"Invalid paddr provided to $deviceName while psel and penable is asserted. paddr: 0x$addr%x"
    )

    val mspi = Module(new spi_top_apb)
    mspi.io.clock := clock
    mspi.io.reset := reset
    io.spi_bundle <> mspi.io.spi
  }

  class ControllerHandler extends Handler("controller", AddressSet.misaligned(0x10001000, 0x1000)) {
    mspi.io.in <> io.in
  }

  class FlashHandler extends Handler("flash", AddressSet.misaligned(0x30000000, 0x10000000)) {
    val s_idle :: s_writeCdr :: s_writeCsr_0 :: s_writeSsr :: s_writeTx1 :: s_writeCsr_1 :: (
      s_readCsr:: s_judge:: s_readRx1 :: s_readRx0 :: s_generateResult :: s_done :: Nil) = Enum(12)
    val state = RegInit(s_idle)
    val begin = Wire(Bool())
    val rwFinished = Wire(Bool())
    val flashAffairsDone = Wire(Bool())
    state := MuxLookup(state, s_idle)(List(
      s_idle -> Mux(begin, s_writeCdr, s_idle),
      s_writeCdr -> Mux(rwFinished, s_writeCsr_0, s_writeCdr),
      s_writeCsr_0 -> Mux(rwFinished, s_writeSsr, s_writeCsr_0),
      s_writeSsr -> Mux(rwFinished, s_writeTx1, s_writeSsr),
      s_writeTx1 -> Mux(rwFinished, s_writeCsr_1, s_writeTx1),
      s_writeCsr_1 -> Mux(rwFinished, s_readCsr, s_writeCsr_1),
      s_readCsr -> Mux(rwFinished, s_judge, s_readCsr),
      s_judge -> Mux(flashAffairsDone, s_readRx1, s_readCsr),
      s_readRx1 -> Mux(rwFinished, s_readRx0, s_readRx1),
      s_readRx0 -> Mux(rwFinished, s_generateResult, s_readRx0),
      s_generateResult -> s_done,
      s_done -> s_idle
    ))

    val apbS_idle :: apbS_read_waitPReady :: apbS_write_waitPReady :: apbS_done :: Nil = Enum(4)
    val apbState = RegInit(apbS_idle)
    val apbReadBegin = Wire(Bool())
    val apbWriteBegin = Wire(Bool())
    val apbPReady = Wire(Bool())
    apbState := MuxLookup(apbState, apbS_idle)(List(
      apbS_idle -> MuxCase(apbS_idle, Seq(
        apbReadBegin -> apbS_read_waitPReady,
        apbWriteBegin -> apbS_write_waitPReady
      )),

      apbS_read_waitPReady -> Mux(apbPReady, apbS_done, apbS_read_waitPReady),

      apbS_write_waitPReady -> Mux(apbPReady, apbS_done, apbS_write_waitPReady),

      apbS_done -> apbS_idle
    ))

    begin := io.in.psel && io.in.penable
    assert(!(begin && io.in.pwrite), "APBSPI: writing to flash is not supported.")

    apbReadBegin := Seq(s_readCsr, s_readRx1, s_readRx0)
      .map(state === _).reduce(_ || _)
    apbWriteBegin := Seq(s_writeCdr, s_writeCsr_0, s_writeSsr, s_writeTx1, s_writeCsr_1)
      .map(state === _).reduce(_ || _)
    val apbPAddr = Wire(UInt(32.W))
    apbPAddr := MuxCase(0.U, Seq(
      (state === s_writeCdr) -> FlashHandler.SPI_CDR_ADDR.U,
      (state === s_writeCsr_0) -> FlashHandler.SPI_CSR_ADDR.U,
      (state === s_writeSsr) -> FlashHandler.SPI_SSR_ADDR.U,
      (state === s_writeTx1) -> FlashHandler.SPI_TX1_ADDR.U,
      (state === s_writeCsr_1) -> FlashHandler.SPI_CSR_ADDR.U,
      (state === s_readCsr) -> FlashHandler.SPI_CSR_ADDR.U,
      (state === s_readRx1) -> FlashHandler.SPI_RX1_ADDR.U,
      (state === s_readRx0) -> FlashHandler.SPI_RX0_ADDR.U
    ))

    mspi.io.in.pwrite := apbWriteBegin
    mspi.io.in.psel := apbReadBegin || apbWriteBegin
    mspi.io.in.paddr := apbPAddr
    mspi.io.in.penable := Seq(apbS_read_waitPReady, apbS_write_waitPReady)
      .map(apbState === _).reduce(_ || _)
    mspi.io.in.pprot := 0.U
    mspi.io.in.pstrb := 0b1111.U
    private def pwdata: UInt = {
      val u32w = 32.W
      val csr = Wire(UInt(u32w))
      val csr1 = Wire(UInt(u32w))
      val flashAddr = Wire(UInt(u32w))
      val value = Wire(UInt(u32w))

      csr := (1.U(u32w) << 13) | (1.U << 9) | 64.U
      csr1 := csr | (1.U << 8)
      flashAddr := io.in.paddr - FlashHandler.FLASH_ADDR.U(u32w)
      value := (0x03.U(u32w) << 24) | (flashAddr & 0x00FFFFFFL.U)

      MuxCase(0.U, Seq(
        (state === s_writeCdr) -> 1.U,
        (state === s_writeCsr_0) -> csr,
        (state === s_writeSsr) -> 1.U,
        (state === s_writeTx1) -> value,
        (state === s_writeCsr_1) -> csr1
      ))
    }
    mspi.io.in.pwdata := Mux(mspi.io.in.pwrite, pwdata, 0.U)
    apbPReady := mspi.io.in.pready
    val prdata = RegInit(0.U(32.W))
    val pslverr = RegInit(false.B)
    when(apbState === apbS_read_waitPReady && apbPReady) {
      prdata := mspi.io.in.prdata
      pslverr := mspi.io.in.pslverr
    }.elsewhen(apbState === apbS_write_waitPReady && apbPReady) {
      prdata := 0.U
      pslverr := mspi.io.in.pslverr
    }
    assert(!pslverr, "APBSPI: pslverr is set inside flash, which is not expected.")
    rwFinished := apbState === apbS_done
    
    val csr_recv = RegInit(0.U(32.W))
    val result = RegInit(0.U(32.W))
    val recv = RegInit(0.U(64.W))

    when(state === s_readCsr && apbState === apbS_done) {
      csr_recv := prdata
    }.elsewhen(state === s_readRx1 && apbState === apbS_done) {
      recv := prdata << 32
    }.elsewhen(state === s_readRx0 && apbState === apbS_done) {
      recv := (recv | prdata) >> 1
    }.elsewhen(state === s_generateResult) {
      result := ((recv & 0x000000FFL.U) << 24) |
                ((recv & 0x0000FF00L.U) << 8) |
                ((recv & 0x00FF0000L.U) >> 8) |
                ((recv & 0xFF000000L.U) >> 24)
    }
    flashAffairsDone := !csr_recv(8)
    io.in.prdata := result
    io.in.pslverr := false.B
    io.in.pready := state === s_done
  }

  object FlashHandler {
    val FLASH_ADDR = 0x30000000L

    val SPI_RX0_ADDR = 0x00
    val SPI_RX1_ADDR = 0x04
    val SPI_RX2_ADDR = 0x08
    val SPI_RX3_ADDR = 0x0C
    val SPI_TX0_ADDR = 0x00
    val SPI_TX1_ADDR = 0x04
    val SPI_TX2_ADDR = 0x08
    val SPI_TX3_ADDR = 0x0C
    val SPI_CSR_ADDR = 0x10
    val SPI_CDR_ADDR = 0x14
    val SPI_SSR_ADDR = 0x18
  }
}
