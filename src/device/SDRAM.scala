package ysyx

import chisel3._
import chisel3.util._
import chisel3.experimental.Analog

import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class SDRAMIO extends Bundle {
  val clk = Output(Bool())
  val cke = Output(Bool())
  val cs  = Output(Bool())
  val ras = Output(Bool())
  val cas = Output(Bool())
  val we  = Output(Bool())
  val a   = Output(UInt(13.W))
  val ba  = Output(UInt(2.W))
  val dqm = Output(UInt(2.W))
  val dq  = Analog(16.W)
}

class sdram_top_axi extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val in = Flipped(new AXI4Bundle(AXI4BundleParameters(addrBits = 32, dataBits = 32, idBits = 4)))
    val sdram = new SDRAMIO
  })
}

class sdram_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val sdram = new SDRAMIO
  })
}

class sdram extends BlackBox {
  val io = IO(Flipped(new SDRAMIO))
}

class sdramChisel extends RawModule {
  val io = IO(Flipped(new SDRAMIO))

  // Wires declared here so they are visible to the TriStateInBuf below,
  // but assigned inside the clock domain block.
  val memRdata = Wire(UInt(16.W))
  val rValid   = Wire(Bool())

  // DQ input from TriStateInBuf — used in write path
  val dqIn = Wire(UInt(16.W))

  // Use io.clk as the clock domain for all sync elements
  withClockAndReset(io.clk.asClock, false.B) {

    // Command decoding: {cs, ras, cas, we}
    val cmd = Cat(io.cs, io.ras, io.cas, io.we)

    val CMD_LMR  = "b0000".U(4.W)
    val CMD_ACT  = "b0011".U(4.W)
    val CMD_WR   = "b0100".U(4.W)
    val CMD_RD   = "b0101".U(4.W)

    // Mode Register — save A[12:0] on LOAD MODE REGISTER command
    val modeReg = RegInit(0.U(13.W))
    when(io.cke && cmd === CMD_LMR) {
      modeReg := io.a
    }

    // Burst length: BL = 1 << modeReg[2:0]  (1, 2, 4, or 8)
    val burstLen = WireDefault(1.U(4.W) << modeReg(2, 0))

    // Bank Row Registers — 4 banks, 13-bit row address each
    val rowReg = SyncReadMem(4, UInt(13.W))
    val rowRegRdata = rowReg.read(io.ba)
    when(io.cke && cmd === CMD_ACT) {
      rowReg.write(io.ba, io.a)
    }

    // READ Pipeline
    val raddr = RegInit(0.U(24.W))
    val rCnt  = RegInit(0.U(4.W))
    when(io.cke && cmd === CMD_RD) {
      rCnt  := burstLen
      raddr := Cat(rowRegRdata, io.ba, io.a(8, 0))
    }.elsewhen(rCnt =/= 0.U) {
      rCnt  := rCnt - 1.U
      raddr := raddr + 1.U
    }
    // rValid registered: samples (rCnt != 0) each cycle, delays by 1.
    // Combined with SyncReadMem's 1-cycle read latency, this yields CL=2.
    val rValidReg = RegInit(false.B)
    rValidReg := (rCnt =/= 0.U)
    rValid := rValidReg

    // WRITE Pipeline
    val waddr = RegInit(0.U(24.W))
    val wCnt  = RegInit(0.U(4.W))
    val wdata = RegInit(0.U(16.W))
    val wmask = RegInit(0.U(2.W))

    when(io.cke && cmd === CMD_WR) {
      wCnt  := burstLen
      waddr := Cat(rowRegRdata, io.ba, io.a(8, 0))
      wdata := dqIn
      wmask := ~io.dqm
    }.elsewhen(wCnt =/= 0.U) {
      wCnt  := wCnt - 1.U
      waddr := waddr + 1.U
      wdata := dqIn
      wmask := ~io.dqm
    }

    // Main Memory — Vec(2, UInt(8.W)) gives byte-granularity write mask
    val mem = SyncReadMem(1 << 24, Vec(2, UInt(8.W)))
    memRdata := Cat(mem.read(raddr)(1), mem.read(raddr)(0))

    when(wCnt =/= 0.U) {
      val wdataVec = Wire(Vec(2, UInt(8.W)))
      wdataVec(0) := wdata(7, 0)
      wdataVec(1) := wdata(15, 8)
      val wmaskVec = Wire(Vec(2, Bool()))
      wmaskVec(0) := wmask(0)
      wmaskVec(1) := wmask(1)
      mem.write(waddr, wdataVec, wmaskVec)
    }
  }

  // TriStateInBuf — combinational, outside clock domain
  dqIn := TriStateInBuf(io.dq, memRdata, rValid)
}

class AXI4SDRAM(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
  val beatBytes = 4
  val node = AXI4SlaveNode(Seq(AXI4SlavePortParameters(
    Seq(AXI4SlaveParameters(
        address       = address,
        executable    = true,
        supportsWrite = TransferSizes(1, beatBytes),
        supportsRead  = TransferSizes(1, beatBytes),
        interleavedId = Some(0))
    ),
    beatBytes  = beatBytes)))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, _) = node.in(0)
    val sdram_bundle = IO(new SDRAMIO)

    val msdram = Module(new sdram_top_axi)
    msdram.io.clock := clock
    msdram.io.reset := reset.asBool
    msdram.io.in <> in
    sdram_bundle <> msdram.io.sdram
  }
}

class APBSDRAM(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val sdram_bundle = IO(new SDRAMIO)

    val msdram = Module(new sdram_top_apb)
    msdram.io.clock := clock
    msdram.io.reset := reset.asBool
    msdram.io.in <> in
    sdram_bundle <> msdram.io.sdram
  }
}
