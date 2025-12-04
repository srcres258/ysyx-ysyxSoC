package ysyx

import chisel3._
import chisel3.util._
import chisel3.experimental.Analog

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class QSPIIO extends Bundle {
  val sck = Output(Bool())
  val ce_n = Output(Bool())
  val dio = Analog(4.W)
}

class psram_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Reset())
    val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val qspi = new QSPIIO
  })
}

class psram extends BlackBox {
  val io = IO(Flipped(new QSPIIO))
}

class psramChisel extends RawModule {
  val io = IO(Flipped(new QSPIIO))

  // --- 1. 双向 I/O 处理: TriStateInBuf ---
  /** 4-bit 双向 (inout) 数据信道 */
  val triStateBuf = Module(new TriStateInBuf(io.dio.getWidth))
  // 连接到该 PSRAM 颗粒的 dio
  triStateBuf.io.dio <> io.dio
  /** PSRAM 接收来自外围设备的数据 (来自 dio) */
  val din = triStateBuf.io.din
  /** PSRAM 输出给外围设备的数据 (到 dio) */
  val dout = triStateBuf.io.dout
  /** 输出使能信号, 控制启用输出 (dout) 还是输入 (din) 信道 */
  val out_en = triStateBuf.io.out_en

  // --- 2. 状态定义, 定义该 PSRAM 状态机模型的各种状态 ---
  object State {
    val nums = 7
    val width = log2Ceil(nums)

    /** 空闲状态, 等待 ce_n 拉低 */
    val idle = 0.U(width.W)
    /** 接收 8-bit 命令 */
    val cmdRecv = 1.U(width.W)

    /** 接收 Quad Read 的 24-bit 地址 (x4, for 6 周期) */
    val quadReadAddr = 2.U(width.W)
    /** 等待 8 个 dummy 周期 */
    val quadReadDummy = 3.U(width.W)
    /** 发送读数据 (x4, 地址自动递增) */
    val quadReadData = 4.U(width.W)

    /** 接收 Quad Write 的 24-bit 地址 (x4, for 6 周期) */
    val quadWriteAddr = 5.U(width.W)
    /** 接收 Quad Write 的数据 (x4, 地址自动递增) */
    val quadWriteData = 6.U(width.W)
  }

  val clock = Wire(Clock())
  clock := io.sck.asClock

  withClock(clock) {
    // --- 3. 定义寄存器, 以y进行边沿检测 (posedge/negedge) ---
    /** 状态寄存器 */
    val stateReg = Reg(UInt(State.width.W))

    // 数据与地址寄存器
    /** 存储接收到的 8-bit 命令 */
    val cmdReg = Reg(UInt(8.W))
    /** 存储 24-bit 地址 */
    val addrReg = Reg(UInt(24.W))

    // 计数器
    /** 命令位计数器 (0~7, 共 8 位) */
    val cmdBitCnt = Reg(UInt(3.W))
    /** 地址 4-bit 组计数器 (0~5, 共 6 组 24 位) */
    val addrCnt = Reg(UInt(3.W))
    /** dummy 周期计数器 (0~7, 共 8 周期) */
    val dummyCnt = Reg(UInt(4.W))

    // 边沿检测 (ce_n 和 sck 的 posedge/negedge)
    val prevCeN = RegNext(io.ce_n)
    /** ce_n negedge: 开始命令接收 */
    val ceNegEdge = !prevCeN && io.ce_n
    /** ce_n posedge: 结束当前操作 */
    val cePosEdge = prevCeN && !io.ce_n
    val prevSck = RegNext(io.sck)
    /** sck posedge: 采样数据 */
    val sckRisingEdge = !prevSck && io.sck

    // --- 4. 控制信号生成 ---
    // out_en: 状态为 quadReadData 时低 (通过 din 接收数据), 否则高 (通过 dout 输出数据)
    out_en := Mux(
      stateReg === State.quadReadData,
      false.B,
      Seq(State.cmdRecv, State.quadWriteAddr, State.quadWriteData)
        .map(stateReg === _)
        .reduce(_ || _)
    )

    // ce_n posedge 时, 重置所有状态与寄存器.
    val reset = Wire(AsyncReset())
    reset := cePosEdge.asAsyncReset
    when(reset.asBool) {
      stateReg := State.idle
      cmdReg := 0.U
      addrReg := 0.U
      cmdBitCnt := 0.U
      addrCnt := 0.U
      dummyCnt := 0.U
    }

    withReset(reset) {
      // --- 5. 定义存储单元与模块输出逻辑 ---
      /** 内存单元, 具有 24-bit 地址, 每个地址可存 4-bit 数据 */
      val mem = Module(new PSRAMMem(useDPIC = true))
      // 读数据, 并在 dout 启用时通过 dout 向模块外输出数据
      val readEnable = Wire(Bool())
      readEnable := stateReg === State.quadReadData
      mem.io.readEnable := readEnable
      mem.io.readAddr := addrReg
      dout := Mux(readEnable, mem.io.readData, 0.U)
      mem.io.writeEnable := false.B
      mem.io.writeAddr := 0.U
      mem.io.writeData := 0.U

      // --- 6. 状态机逻辑 ---
      when(stateReg === State.idle) {
        // 空闲状态: 等待 ce_n 拉低
        when(ceNegEdge) {
          stateReg := State.cmdRecv
          cmdReg := 0.U
          addrReg := 0.U
          cmdBitCnt := 0.U
        }
      }.elsewhen(stateReg === State.cmdRecv) {
        // 接收 8-bit 命令: 左移采样, 完成后根据命令跳转
        when(sckRisingEdge) {
          cmdReg := cmdReg(6, 0) ## din
          cmdBitCnt := cmdBitCnt + 1.U
          // 当接收完 8 位命令时
          when(cmdBitCnt === 7.U) {
            assert(
              Seq("hEB", "h38").map(cmdReg === _.U).reduce(_ || _),
              "Unknown command passed to PSRAM."
            )
            when(cmdReg === "hEB".U) {
              // Quad Read 命令
              stateReg := State.quadReadAddr
            }.elsewhen(cmdReg === "h38".U) {
              // Quad Write 命令
              stateReg := State.quadWriteAddr
            }.otherwise {
              // 未知命令, 回到空闲状态
              stateReg := State.idle
            }
            cmdBitCnt := 0.U
          }
        }
      }.elsewhen(stateReg === State.quadReadAddr) {
        // Quad Read, 接收 24-bit 地址 (x4, for 6 周期)
        when(sckRisingEdge) {
          addrReg := (addrReg << 4.U) | din // 左移 4 位, 接收地址的 4-bit 组
          addrCnt := addrCnt + 1.U
          when(addrCnt === 5.U) {
            // 已经 6 个周期; 已经接收完了 24-bit 地址
            stateReg := State.quadReadDummy
            addrCnt := 0.U
          }
        }
      }.elsewhen(stateReg === State.quadReadDummy) {
        // QuadRead, 等待 8 个 dummy 周期 (x4)
        when(sckRisingEdge) {
          dummyCnt := dummyCnt + 1.U
          when(dummyCnt === 7.U) {
            // 已经 8 个周期; dummy 周期全部结束了
            stateReg := State.quadReadData
            dummyCnt := 0.U
          }
        }
      }.elsewhen(stateReg === State.quadWriteAddr) {
        // Quad Write, 接收 24-bit 地址 (x4, for 6 周期)
        when(sckRisingEdge) {
          addrReg := (addrReg << 4.U) | din // 左移 4 位, 接收地址的 4-bit 组
          addrCnt := addrCnt + 1.U
          when(addrCnt === 5.U) {
            // 已经 6 个周期; 已经接收完了 24-bit 地址
            stateReg := State.quadWriteData
            addrCnt := 0.U
          }
        }
      }.elsewhen(stateReg === State.quadWriteData) {
        // Quad Write, 接收数据并写入存储单元 (x4, 地址自动递增)
        when(sckRisingEdge) {
          // 将 4-bit 数据写入当前地址
          mem.io.writeEnable := true.B
          mem.io.writeAddr := addrReg
          mem.io.writeData := din
          // 地址自动递增
          addrReg := addrReg + 1.U
          when(cePosEdge) {
            // ce_n 拉高, 结束写入
            stateReg := State.idle
          }
        }
      }
    }
  }
}

class PSRAMMemBundle extends Bundle {
  val readAddr = Input(UInt(24.W))
  val readEnable = Input(Bool())
  val readData = Output(UInt(8.W))

  val writeAddr = Input(UInt(24.W))
  val writeEnable = Input(Bool())
  val writeData = Input(UInt(8.W))
}

class PSRAMMemImplDPIC extends BlackBox with HasBlackBoxInline {
  val io = IO(new PSRAMMemBundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
  })

  setInline(
    "PSRAMMemImplDPIC.v",
    """module PSRAMMemImplDPIC(
      |    input clock,
      |    input reset,
      |
      |    input [23:0] readAddr,
      |    input readEnable,
      |    output reg [7:0] readData /* verilator public_flat */,
      |
      |    input [23:0] writeAddr,
      |    input writeEnable,
      |    input [7:0] writeData
      |);
      |    import "DPI-C" function void psram_read(input [23:0] raddr, output byte rdata);
      |    import "DPI-C" function void psram_write(input [23:0] waddr, input byte wdata);
      |
      |    always @(posedge clock) begin
      |        if (readEnable) psram_read(readAddr, readData);
      |        else readData = 0;
      |
      |        if (writeEnable) psram_write(writeAddr, writeData);
      |    end
      |endmodule
    """.stripMargin
  )
}

class PSRAMMemImplChisel extends Module {
  val io = IO(new PSRAMMemBundle)

  val mem = SyncReadMem(1 << 24, UInt(8.W))
  io.readData := Mux(io.readEnable, mem.read(io.readAddr), 0.U)
  when(io.writeEnable) {
    mem.write(io.writeAddr, io.writeData)
  }
}

class PSRAMMem(useDPIC: Boolean) extends Module {
  val io = IO(new PSRAMMemBundle)

  if (useDPIC) {
    val impl = Module(new PSRAMMemImplDPIC)
    impl.io.clock := clock.asBool
    impl.io.reset := reset.asBool
    impl.io.readAddr := io.readAddr
    impl.io.readEnable := io.readEnable
    io.readData := impl.io.readData
    impl.io.writeAddr := io.writeAddr
    impl.io.writeEnable := io.writeEnable
    impl.io.writeData := io.writeData
  } else {
    val impl = Module(new PSRAMMemImplChisel)
    impl.io <> io
  }
}

class APBPSRAM(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val qspi_bundle = IO(new QSPIIO)

    val mpsram = Module(new psram_top_apb)
    mpsram.io.clock := clock
    mpsram.io.reset := reset
    mpsram.io.in <> in
    qspi_bundle <> mpsram.io.qspi
  }
}
