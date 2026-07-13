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
  val dqm = Output(UInt(4.W))
  val dq  = Analog(32.W)
  val dataOut = Input(UInt(32.W))
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
// DPI-backed particle memory.  The NPC backend owns the canonical byte store,
// so the SoC model reads/writes it through DPI instead of private RAM state.
// ---------------------------------------------------------------------
class SDRAMMemBundle extends Bundle {
  val readAddr = Input(UInt(27.W))
  val readEnable = Input(Bool())
  val readData = Output(UInt(16.W))

  val writeAddr = Input(UInt(27.W))
  val writeEnable = Input(Bool())
  val writeData = Input(UInt(16.W))
  val writeMask = Input(UInt(2.W))
}

class SDRAMMemImplDPIC extends BlackBox with HasBlackBoxInline {
  val io = IO(new SDRAMMemBundle {
    val clock = Input(Bool())
  })

  setInline(
    "SDRAMMemImplDPIC.v",
    """module SDRAMMemImplDPIC(
      |    input clock,
      |    input [26:0] readAddr,
      |    input readEnable,
      |    output reg [15:0] readData,
      |
      |    input [26:0] writeAddr,
      |    input writeEnable,
      |    input [15:0] writeData,
      |    input [1:0] writeMask
      |);
      |    import "DPI-C" function void sdram_read(input int addr, output byte data);
      |    import "DPI-C" function void sdram_write(input int addr, input byte data);
      |
      |    reg [7:0] lo;
      |    reg [7:0] hi;
      |
      |    always @(posedge clock) begin
      |        if (readEnable) begin
      |            sdram_read(readAddr, lo);
      |            sdram_read(readAddr + 1, hi);
      |            readData <= {hi, lo};
      |        end else begin
      |            readData <= 0;
      |        end
      |
      |        if (writeEnable) begin
      |            if (!writeMask[0]) sdram_write(writeAddr, writeData[7:0]);
      |            if (!writeMask[1]) sdram_write(writeAddr + 1, writeData[15:8]);
      |        end
      |    end
      |endmodule
    """.stripMargin
  )
}

// ---------------------------------------------------------------------
// Internal MT48LC16M16A2 x16 particle core.
//
// This core models one physical SDRAM particle in the digital domain:
// - 4 banks × 8192 rows × 512 cols × 16 bits = 32 MB
// - bank-open state and active-row tracking
// - LOAD MODE REGISTER / ACTIVE / READ / WRITE / PRECHARGE / AUTO REFRESH
// - COMMAND INHIBIT and NOP as protocol-visible no-ops
//
// PRECHARGE and AUTO REFRESH keep their protocol side effects, but do not
// model DRAM-cell electrical retention. This matches the ysyx lecture's
// intended simulation scope while staying command-correct.
// ---------------------------------------------------------------------
class SdramParticleCore(addrShift: Int, laneOffsetBytes: Int, baseOffsetBytes: Long) extends Module {
  val io = IO(new Bundle {
    val cke = Input(Bool())
    val cs  = Input(Bool())
    val ras = Input(Bool())
    val cas = Input(Bool())
    val we  = Input(Bool())
    val a   = Input(UInt(13.W))
    val ba  = Input(UInt(2.W))

    val dqm       = Input(UInt(2.W))
    val writeData = Input(UInt(16.W))

    val readData  = Output(UInt(16.W))
    val readDrive = Output(Bool())
  })

  val cmd = Cat(io.cs, io.ras, io.cas, io.we)

  val CMD_LMR = "b0000".U(4.W)
  val CMD_AR  = "b0001".U(4.W)
  val CMD_PRE = "b0010".U(4.W)
  val CMD_ACT = "b0011".U(4.W)
  val CMD_WR  = "b0100".U(4.W)
  val CMD_RD  = "b0101".U(4.W)
  val CMD_BT  = "b0110".U(4.W)
  val CMD_NOP = "b0111".U(4.W)

  val commandInhibit = io.cke && io.cs
  val commandAccept  = io.cke && !io.cs

  val isLmr = commandAccept && cmd === CMD_LMR
  val isAr  = commandAccept && cmd === CMD_AR
  val isPre = commandAccept && cmd === CMD_PRE
  val isAct = commandAccept && cmd === CMD_ACT
  val isWr  = commandAccept && cmd === CMD_WR
  val isRd  = commandAccept && cmd === CMD_RD
  val isBt  = commandAccept && cmd === CMD_BT
  val isNop = commandAccept && cmd === CMD_NOP

  val modeReg = RegInit(0.U(13.W))
  val burstLen = MuxLookup(modeReg(2, 0), 1.U(4.W))(Seq(
    0.U -> 1.U,
    1.U -> 2.U,
    2.U -> 4.U,
    3.U -> 8.U
  ))
  val casLatency = MuxLookup(modeReg(6, 4), 2.U(2.W))(Seq(
    "b010".U -> 2.U,
    "b011".U -> 3.U
  ))

  val bankOpen = RegInit(VecInit(Seq.fill(4)(false.B)))
  val openRow  = RegInit(VecInit(Seq.fill(4)(0.U(13.W))))

  val allBanksIdle = !bankOpen.asUInt.orR
  val targetBankOpen = bankOpen(io.ba)
  val targetRow = openRow(io.ba)
  val colAddr = io.a(8, 0)

  def mkAddr(row: UInt, bank: UInt, col: UInt): UInt = Cat(row, bank, col)

  val readAddr      = RegInit(0.U(24.W))
  val readRemain    = RegInit(0.U(4.W))
  val readBank      = RegInit(0.U(2.W))
  val readAutoPre   = RegInit(false.B)

  val writeAddr     = RegInit(0.U(24.W))
  val writeRemain   = RegInit(0.U(4.W))
  val writeBank     = RegInit(0.U(2.W))
  val writeAutoPre  = RegInit(false.B)

  val readIssue     = WireDefault(false.B)
  val readIssueAddr = WireDefault(0.U(24.W))

  val writeFire     = WireDefault(false.B)
  val writeFireAddr = WireDefault(0.U(24.W))
  val writeFireData = WireDefault(0.U(16.W))
  val writeFireMask = WireDefault(0.U(2.W))

  val mem = Module(new SDRAMMemImplDPIC)
  val baseOffset = baseOffsetBytes.U(27.W)
  val laneOffset = laneOffsetBytes.U(27.W)
  mem.io.clock := clock.asBool
  mem.io.readAddr := baseOffset + Cat(readIssueAddr, 0.U(addrShift.W)) + laneOffset
  mem.io.readEnable := readIssue
  mem.io.writeAddr := baseOffset + Cat(writeFireAddr, 0.U(addrShift.W)) + laneOffset
  mem.io.writeEnable := writeFire
  mem.io.writeData := writeFireData
  mem.io.writeMask := writeFireMask

  when(isLmr) {
    assert(allBanksIdle, "SDRAM LMR requires all banks idle")
    assert(io.a(2, 0) <= 3.U, "SDRAM unsupported burst length in mode register")
    assert(io.a(6, 4) === "b010".U || io.a(6, 4) === "b011".U,
      "SDRAM unsupported CAS latency in mode register")
    modeReg := io.a
  }

  when(isAct) {
    assert(!targetBankOpen, "SDRAM ACTIVE to already-open bank")
    openRow(io.ba) := io.a
    bankOpen(io.ba) := true.B
  }

  when(isPre) {
    when(io.a(10)) {
      bankOpen.foreach(_ := false.B)
    }.otherwise {
      bankOpen(io.ba) := false.B
    }
  }

  when(isAr) {
    assert(allBanksIdle, "SDRAM AUTO REFRESH requires all banks idle")
  }

  when(isBt) {
    readRemain := 0.U
    writeRemain := 0.U
    when(readAutoPre)  { bankOpen(readBank)  := false.B }
    when(writeAutoPre) { bankOpen(writeBank) := false.B }
    readAutoPre  := false.B
    writeAutoPre := false.B
  }

  val readCmdLegal = isRd && targetBankOpen
  val writeCmdLegal = isWr && targetBankOpen

  when(isRd) {
    assert(targetBankOpen, "SDRAM READ requires ACTIVE row in target bank")
  }
  when(isWr) {
    assert(targetBankOpen, "SDRAM WRITE requires ACTIVE row in target bank")
  }

  when(readCmdLegal) {
    val startAddr = mkAddr(targetRow, io.ba, colAddr)
    readIssue := true.B
    readIssueAddr := startAddr
    readRemain := burstLen - 1.U
    readAddr := startAddr + 1.U
    readBank := io.ba
    readAutoPre := io.a(10)
    when(burstLen === 1.U && io.a(10)) {
      bankOpen(io.ba) := false.B
      readAutoPre := false.B
    }
  }.elsewhen(readRemain =/= 0.U) {
    readIssue := true.B
    readIssueAddr := readAddr
    when(readRemain === 1.U) {
      when(readAutoPre) {
        bankOpen(readBank) := false.B
      }
      readAutoPre := false.B
    }
    readRemain := readRemain - 1.U
    readAddr := readAddr + 1.U
  }

  when(writeCmdLegal) {
    val startAddr = mkAddr(targetRow, io.ba, colAddr)
    writeFire := true.B
    writeFireAddr := startAddr
    writeFireData := io.writeData
    writeFireMask := io.dqm
    writeRemain := burstLen - 1.U
    writeAddr := startAddr + 1.U
    writeBank := io.ba
    writeAutoPre := io.a(10)
    when(burstLen === 1.U && io.a(10)) {
      bankOpen(io.ba) := false.B
      writeAutoPre := false.B
    }
  }.elsewhen(writeRemain =/= 0.U) {
    writeFire := true.B
    writeFireAddr := writeAddr
    writeFireData := io.writeData
    writeFireMask := io.dqm
    when(writeRemain === 1.U) {
      when(writeAutoPre) {
        bankOpen(writeBank) := false.B
      }
      writeAutoPre := false.B
    }
    writeRemain := writeRemain - 1.U
    writeAddr := writeAddr + 1.U
  }

  val rawReadData = mem.io.readData
  val readValidD1 = RegNext(readIssue, false.B)
  val readValidD2 = RegNext(readValidD1, false.B)
  val readValidD3 = RegNext(readValidD2, false.B)

  val readDataStage1 = RegInit(0.U(16.W))
  val readDataStage2 = RegInit(0.U(16.W))

  when(readValidD1) {
    readDataStage1 := rawReadData
  }
  when(readValidD2) {
    readDataStage2 := readDataStage1
  }

  io.readData := Mux(casLatency === 3.U, readDataStage2, readDataStage1)
  io.readDrive := Mux(casLatency === 3.U, readValidD3, readValidD2)

  dontTouch(commandInhibit)
  dontTouch(isNop)
}

// ---------------------------------------------------------------------
// MT48LC16M16A2 SDRAM simulation behavior model (Chisel)
//
// laneCount=1 : one x16 particle using the low 16 bits of SDRAMIO.dq
// laneCount=2 : two x16 particles in bit-extension mode, forming a 32-bit
//               SDRAM controller channel while preserving per-particle state
// ---------------------------------------------------------------------
class sdramChisel(laneCount: Int = 1, baseOffsetBytes: Long = 0L) extends RawModule {
  require(laneCount == 1 || laneCount == 2, "sdramChisel laneCount must be 1 or 2")

  val io = IO(Flipped(new SDRAMIO))

  val dqOut32 = Wire(UInt(32.W))
  val dqIn32  = Wire(UInt(32.W))
  val dqOe    = Wire(Bool())

  val laneReadData  = Wire(Vec(2, UInt(16.W)))
  val laneReadDrive = Wire(Vec(2, Bool()))
  laneReadData.foreach(_ := 0.U)
  laneReadDrive.foreach(_ := false.B)

  withClockAndReset(io.clk.asClock, false.B) {
    val lanes = Seq.tabulate(laneCount) { idx =>
      val addrShift = if (laneCount == 2) 2 else 1
      Module(new SdramParticleCore(addrShift, idx * 2, baseOffsetBytes))
    }

    for (idx <- 0 until laneCount) {
      val lane = lanes(idx)
      val dqmLo = idx * 2
      val dqLo  = idx * 16

      lane.io.cke := io.cke
      lane.io.cs  := io.cs
      lane.io.ras := io.ras
      lane.io.cas := io.cas
      lane.io.we  := io.we
      lane.io.a   := io.a
      lane.io.ba  := io.ba
      lane.io.dqm := io.dqm(dqmLo + 1, dqmLo)
      lane.io.writeData := dqIn32(dqLo + 15, dqLo)

      laneReadData(idx) := lane.io.readData
      laneReadDrive(idx) := lane.io.readDrive
    }

    if (laneCount == 2) {
      assert(lanes(0).io.readDrive === lanes(1).io.readDrive,
        "Bit-extended SDRAM lanes must drive read data in lockstep")
    }
  }

  dqOut32 := Cat(laneReadData(1), laneReadData(0))
  dqOe := laneReadDrive.asUInt.orR
  dqIn32 := TriStateInBuf(io.dq, dqOut32, dqOe)

  // Read data bypass — works around Verilator hierarchical tri-state issue.
  io.dataOut := dqOut32
}

// =====================================================================
// Diplomacy wrappers (for SoC integration)
// =====================================================================

// =====================================================================
// SDRAM address constants — computed from Config at Chisel elaboration time
// =====================================================================
object SDRAMAddr {
  val SDRAM_BASE = 0xa0000000L

  /** Compute the AddressSet list based on Config.
    *
    * Returns 1 or 2 AddressSets corresponding to each SDRAM controller channel.
    * Each channel covers 64MB when sdramBitExt=true (32-bit data), or 32MB
    * when sdramBitExt=false (16-bit data).
    *
    * Total capacity: channelCount × channelSize.
    */
  def addressSets: Seq[AddressSet] = {
    val channelSize = if (Config.sdramBitExt) 0x4000000L else 0x2000000L // 64MB or 32MB
    val channelCount = if (Config.sdramBitExt && Config.sdramWordExt) 2 else 1
    // Use explicit AddressSet(base, mask) to guarantee non-overlapping
    // regions.  Diplomacy convention: mask=1 bits are "don't care",
    // mask=0 bits must match.  mask = channelSize-1 ignores the low
    // address bits that vary within a channel, forcing the high bits
    // (including the channel-select bit) to match the base address.
    (0 until channelCount).map { ch =>
      AddressSet(SDRAM_BASE + ch * channelSize, channelSize - 1)
    }
  }
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
