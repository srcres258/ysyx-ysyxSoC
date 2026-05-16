package ysyx

import chisel3._
import chisel3.util._
import chisel3.experimental.Analog

import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

// ---------------------------------------------------------------------
// SDRAM I/O Bundle — MT48LC16M16A2 interface
// ---------------------------------------------------------------------
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
  val dataOut = Input(UInt(16.W))
}

// ---------------------------------------------------------------------
// BlackBox wrappers for the Verilog SDRAM controller
// ---------------------------------------------------------------------
class sdram_top_axi extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val in = Flipped(new AXI4Bundle(AXI4BundleParameters(
      addrBits = 32, dataBits = 32, idBits = 4)))
    val sdram = new SDRAMIO
  })
}

class sdram_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val in = Flipped(new APBBundle(APBBundleParameters(
      addrBits = 32, dataBits = 32)))
    val sdram = new SDRAMIO
  })
}

class sdram extends BlackBox {
  val io = IO(Flipped(new SDRAMIO))
}

// ---------------------------------------------------------------------
// MT48LC16M16A2 SDRAM simulation behavior model (Chisel)
//
// Specifications:
//   4 banks x 8192 rows x 512 cols x 16 bits = 256 Mbit = 32 MB
//   Internal address: {row[12:0], bank[1:0], col[8:0]} — 24 bits
//
// Supported commands:
//   LOAD MODE REGISTER — stores CAS Latency / Burst Length
//   ACTIVE             — opens a row in the specified bank
//   READ               — reads from active row (data after CL cycles)
//   WRITE              — writes to active row (data same cycle as cmd)
//   PRECHARGE / REFRESH / BURST TERMINATE — treated as NOP
// ---------------------------------------------------------------------
class sdramChisel extends RawModule {
  val io = IO(Flipped(new SDRAMIO))

  // Cross-domain signals (declared at RawModule scope for TriStateInBuf)
  val memRdata = Wire(UInt(16.W))
  val rValid   = Wire(Bool())
  val dqIn     = Wire(UInt(16.W))

  // ===================================================================
  // Clock domain: io.clk (SDRAM clock = ~system clock)
  // ===================================================================
  withClockAndReset(io.clk.asClock, false.B) {

    // ---------------------------------------------------------------
    // Command decoding: {cs, ras, cas, we} (all active low)
    // ---------------------------------------------------------------
    val cmd = Cat(io.cs, io.ras, io.cas, io.we)

    val CMD_LMR  = "b0000".U(4.W)  // LOAD MODE REGISTER
    val CMD_AR   = "b0001".U(4.W)  // AUTO REFRESH       -> NOP
    val CMD_PRE  = "b0010".U(4.W)  // PRECHARGE           -> NOP
    val CMD_ACT  = "b0011".U(4.W)  // ACTIVE
    val CMD_WR   = "b0100".U(4.W)  // WRITE
    val CMD_RD   = "b0101".U(4.W)  // READ
    val CMD_BT   = "b0110".U(4.W)  // BURST TERMINATE     -> NOP
    val CMD_NOP  = "b0111".U(4.W)  // NOP

    val cmdValid = io.cke  // Commands only valid when CKE is high

    // ---------------------------------------------------------------
    // Mode Register
    //   A[2:0]  — Burst Length  (001=2, 010=4, 011=8)
    //   A[6:4]  — CAS Latency   (010=2, 011=3)
    // ---------------------------------------------------------------
    val modeReg = RegInit(0.U(13.W))
    when(cmdValid && cmd === CMD_LMR) {
      modeReg := io.a
    }

    val burstLen = MuxLookup(modeReg(2, 0), 1.U(4.W))(Seq(
      0.U  ->  1.U,
      1.U  ->  2.U,
      2.U  ->  4.U,
      3.U  ->  8.U
    ))

    val casLatency = Mux(modeReg(6, 4) === "b011".U, 3.U, 2.U)

    // ---------------------------------------------------------------
    // Bank Row Registers — 4 banks, each stores the active row
    // ---------------------------------------------------------------
    val rowReg = SyncReadMem(4, UInt(13.W))
    val rowRegRdata = rowReg.read(io.ba)
    when(cmdValid && cmd === CMD_ACT) {
      rowReg.write(io.ba, io.a)
    }

    // ---------------------------------------------------------------
    // Main memory array — 16M x 16-bit = 32 MB
    // ---------------------------------------------------------------
    val memDepth = 1 << 24  // 16,777,216
    val mem = SyncReadMem(memDepth, Vec(2, UInt(8.W)))

    // ===============================================================
    // Read Pipeline
    // ===============================================================
    val rAddr = RegInit(0.U(24.W))
    val rCnt  = RegInit(0.U(4.W))

    when(cmdValid && cmd === CMD_RD) {
      rAddr := Cat(rowRegRdata, io.ba, io.a(8, 0))
      rCnt  := burstLen
    }.elsewhen(rCnt =/= 0.U) {
      rAddr := rAddr + 1.U
      rCnt  := rCnt - 1.U
    }

    // Synchronous read memory (1 cycle latency)
    val rawReadData = mem.read(rAddr)
    memRdata := Cat(rawReadData(1), rawReadData(0))

    // Read valid delay register — combined with SyncReadMem's 1-cycle
    // latency to achieve CAS Latency = 2 timing.
    //   Cycle T:   READ cmd, rCnt <- burstLen
    //   Cycle T+1: rCnt > 0, issue mem.read(A0), rCnt <- burstLen-1
    //   Cycle T+2: A0 data ready, rValid = true -> drive DQ
    //   Cycle T+3: A1 data ready (burst continues)
    val rValidReg = RegInit(false.B)
    rValidReg := (rCnt =/= 0.U)
    rValid := rValidReg

    // ===============================================================
    // Write Pipeline
    // ===============================================================
    val wAddr = RegInit(0.U(24.W))
    val wCnt  = RegInit(0.U(4.W))

    val we = WireDefault(false.B)
    val wa = WireDefault(0.U(24.W))
    val wd = WireDefault(0.U(16.W))
    val wq = WireDefault(0.U(2.W))

    when(cmdValid && cmd === CMD_WR) {
      // First beat: data present on DQ bus same cycle as WRITE command
      we    := true.B
      wa    := Cat(rowRegRdata, io.ba, io.a(8, 0))
      wd    := dqIn
      wq    := io.dqm
      wCnt  := burstLen - 1.U
      wAddr := Cat(rowRegRdata, io.ba, io.a(8, 0)) + 1.U
    }.elsewhen(wCnt =/= 0.U) {
      // Subsequent beats
      we    := true.B
      wa    := wAddr
      wd    := dqIn
      wq    := io.dqm
      wCnt  := wCnt - 1.U
      wAddr := wAddr + 1.U
    }

    // Single write port with byte masking
    when(we) {
      val ww = Wire(Vec(2, UInt(8.W)))
      ww(0) := wd(7, 0)    // low byte
      ww(1) := wd(15, 8)   // high byte

      val mm = Wire(Vec(2, Bool()))
      // DQM active high: 1 = mask (do not write)
      // mask = ~dqm  (true = write, false = mask)
      mm(0) := ~wq(0)
      mm(1) := ~wq(1)

      mem.write(wa, ww, mm)
    }
  }

  // ===================================================================
  // Tri-state data bus — outside clock domain (combinational)
  // ===================================================================
  dqIn := TriStateInBuf(io.dq, memRdata, rValid)

  // Read data bypass — works around Verilator hierarchical tri-state issue
  io.dataOut := memRdata
}

// =====================================================================
// Diplomacy wrappers (for SoC integration)
// =====================================================================

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
