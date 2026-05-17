package ysyx

import chisel3._
import chisel3.util._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class PS2IO extends Bundle {
  val clk = Input(Bool())
  val data = Input(Bool())
}

class PS2CtrlIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val ps2 = new PS2IO
}

class ps2_top_apb extends BlackBox {
  val io = IO(new PS2CtrlIO)
}

class ps2Chisel extends Module {
  val io = IO(new PS2CtrlIO)

  // --- Clock domain crossing: two-stage synchronizer for ps2_clk ---
  val ps2_clk_sync0 = withClockAndReset(io.clock, io.reset) { RegInit(true.B) }
  val ps2_clk_sync1 = withClockAndReset(io.clock, io.reset) { RegInit(true.B) }
  val ps2_data_sync = withClockAndReset(io.clock, io.reset) { RegInit(true.B) }

  ps2_clk_sync0 := io.ps2.clk
  ps2_clk_sync1 := ps2_clk_sync0
  ps2_data_sync := io.ps2.data

  // Falling edge detector for synchronized ps2_clk
  val ps2_clk_prev = RegInit(true.B)
  ps2_clk_prev := ps2_clk_sync1
  val ps2_clk_falling = ps2_clk_prev && !ps2_clk_sync1

  // --- PS/2 frame receive state machine ---
  val sIdle :: sData :: sParity :: sStop :: Nil = Enum(4)
  val state = RegInit(sIdle)
  val bitCount = RegInit(0.U(3.W))     // 0..7 data bits
  val shiftReg = RegInit(0.U(8.W))     // received data byte (LSB first)
  val parityAcc = RegInit(false.B)     // accumulated parity (over data bits)

  // FIFO: 16-entry queue for buffering received scan codes
  val fifo = Module(new Queue(UInt(8.W), 16))
  fifo.io.enq.valid := false.B
  fifo.io.enq.bits := shiftReg

  when(ps2_clk_falling) {
    switch(state) {
      is(sIdle) {
        // Start bit should be 0
        when(!ps2_data_sync) {
          state     := sData
          bitCount  := 0.U
          shiftReg  := 0.U
          parityAcc := false.B
        }
      }
      is(sData) {
        // LSB first: shift right, new bit goes to MSB
        shiftReg  := Cat(ps2_data_sync, shiftReg(7, 1))
        parityAcc := parityAcc ^ ps2_data_sync
        bitCount  := bitCount + 1.U
        when(bitCount === 7.U) {
          state := sParity
        }
      }
      is(sParity) {
        // Odd parity: parity bit should make total number of 1s odd
        // parityAcc holds XOR of all 8 data bits
        // So (parityAcc ^ parity_bit) should be 1 for odd parity
        when((parityAcc ^ ps2_data_sync) === 1.U) {
          state := sStop
        }.otherwise {
          // Parity error -> discard frame, return to idle
          state := sIdle
        }
      }
      is(sStop) {
        // Stop bit should be 1
        when(ps2_data_sync) {
          // Valid frame -> push scan code to FIFO
          fifo.io.enq.valid := true.B
        }
        state := sIdle
      }
    }
  }

  // --- APB read interface ---
  val byteOffset = io.in.paddr(3, 0)
  val readActive = !io.in.pwrite && io.in.psel && io.in.penable

  // Default read data = 0
  io.in.prdata := 0.U

  // Read from offset 0x0: return FIFO byte (or 0 if empty)
  when(readActive && byteOffset === 0x0.U) {
    io.in.prdata := Cat(0.U(24.W), Mux(fifo.io.deq.valid, fifo.io.deq.bits, 0.U(8.W)))
  }

  // FIFO dequeue: pop when a valid read occurs at offset 0 and FIFO is non-empty
  fifo.io.deq.ready := readActive && byteOffset === 0x0.U && fifo.io.deq.valid

  // APB handshake
  io.in.pready  := io.in.psel && io.in.penable
  io.in.pslverr := false.B
}

class APBKeyboard(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val ps2_bundle = IO(new PS2IO)

    // Use Chisel implementation instead of Verilog BlackBox
    val mps2 = Module(new ps2Chisel)
    mps2.io.clock := clock
    mps2.io.reset := reset
    mps2.io.in <> in
    ps2_bundle <> mps2.io.ps2
  }
}
