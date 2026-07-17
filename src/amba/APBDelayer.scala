package ysyx

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.amba._
import freechips.rocketchip.amba.apb._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class APBDelayerIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Reset())
  val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val out = new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32))
}

class apb_delayer extends BlackBox {
  val io = IO(new APBDelayerIO)
}

class APBDelayerChisel extends Module {
  val io = IO(new APBDelayerIO)

  // Fixed-point constants for calibrated delay: RatioScaled / Scale = 192 / 128 = 1.5
  val ScaleBits = 7
  val Scale = 128
  val RatioScaled = 192

  // FSM states — s_setup/s_access preserve APB sequencing downstream
  val s_idle :: s_setup :: s_access :: s_delay :: Nil = Enum(4)
  val state = RegInit(s_idle)

  // Cached upstream request fields (mirrors SPIInBundle pattern from SPI.scala)
  val req_pwrite = RegInit(false.B)
  val req_paddr  = RegInit(0.U(32.W))
  val req_pprot  = RegInit(0.U(3.W))
  val req_pwdata = RegInit(0.U(32.W))
  val req_pstrb  = RegInit(0.U(4.W))

  // Cached downstream response fields
  val resp_prdata  = RegInit(0.U(32.W))
  val resp_pslverr = RegInit(false.B)

  // Transaction cycle counter — counts elapsed ACCESS cycles (penable=1 downstream)
  val transCycles = RegInit(0.U(8.W))
  val transCyclesNext = Wire(UInt(8.W))
  transCyclesNext := Mux(state === s_access, transCycles + 1.U, 0.U)
  transCycles := transCyclesNext

  // Calibrated extra delay: ceil((t1 - t0) * RatioScaled / Scale) using next-cycle count
  val calibratedTotal = (transCyclesNext * RatioScaled.U + (Scale - 1).U) / Scale.U
  val extraCycles = Wire(UInt(8.W))
  extraCycles := calibratedTotal - transCyclesNext

  // Delay countdown register
  val delayCounter = RegInit(0.U(8.W))

  // ── FSM transitions ──

  when (state === s_idle) {
    when (io.in.psel) {
      req_pwrite := io.in.pwrite
      req_paddr  := io.in.paddr
      req_pprot  := io.in.pprot
      req_pwdata := io.in.pwdata
      req_pstrb  := io.in.pstrb
      state      := s_setup
    }
  }

  when (state === s_setup) {
    state := s_access
  }

  when (state === s_access) {
    when (io.out.pready) {
      resp_prdata  := io.out.prdata
      resp_pslverr := io.out.pslverr
      when (extraCycles === 0.U) {
        state := s_idle
      }.otherwise {
        state         := s_delay
        delayCounter := extraCycles
      }
    }
  }

  when (state === s_delay) {
    delayCounter := delayCounter - 1.U
    when (delayCounter === 1.U) {
      state := s_idle
    }
  }

  // ── Downstream (io.out) signals ──
  io.out.psel    := state === s_setup || state === s_access
  io.out.penable := state === s_access
  io.out.pwrite  := req_pwrite
  io.out.paddr   := req_paddr
  io.out.pprot   := req_pprot
  io.out.pwdata  := req_pwdata
  io.out.pstrb   := req_pstrb

  // ── Upstream (io.in) response signals ──
  io.in.pready := MuxCase(false.B, Seq(
    (state === s_access && io.out.pready && extraCycles === 0.U) -> true.B,
    (state === s_delay  && delayCounter === 1.U)                 -> true.B
  ))
  io.in.prdata  := Mux(state === s_delay, resp_prdata, 0.U)
  io.in.pslverr := Mux(state === s_delay, resp_pslverr, false.B)

  // ── Protocol assertions and reset guardrails ──

  // Rising-edge detector for upstream psel — catches new transaction attempts while busy
  val prevPsel = RegNext(io.in.psel, false.B)
  val pselRising = io.in.psel && !prevPsel

  // Busy condition: upstream ACCESS phase without our response, and we have captured state
  val inAccess = io.in.psel && io.in.penable && !io.in.pready
  val busy = state =/= s_idle && inAccess

  // ── Request stability: fields must not change during active transfer ──
  when (busy) {
    assert(req_pwrite === io.in.pwrite, "APBDelayer: pwrite changed during active transfer")
    assert(req_paddr  === io.in.paddr,  "APBDelayer: paddr changed during active transfer")
    assert(req_pprot  === io.in.pprot,  "APBDelayer: pprot changed during active transfer")
    assert(req_pwdata === io.in.pwdata, "APBDelayer: pwdata changed during active transfer")
    assert(req_pstrb  === io.in.pstrb,  "APBDelayer: pstrb changed during active transfer")
  }

  // ── Single-outstanding: reject new transaction while busy ──
  when (state =/= s_idle) {
    assert(!pselRising, "APBDelayer: new psel assertion while busy -- single-outstanding violation")
  }

  // ── Downstream idle during delay phase ──
  when (state === s_delay) {
    assert(!io.out.psel,    "APBDelayer: downstream psel asserted during delay phase")
    assert(!io.out.penable, "APBDelayer: downstream penable asserted during delay phase")
  }

  // ── Downstream protocol self-check: penable must not be asserted without psel ──
  assert(!io.out.penable || io.out.psel, "APBDelayer: downstream penable without psel")

  // ── Reset hygiene: all state/caches must clear to defaults during reset ──
  when (reset.asBool) {
    assert(state        === s_idle,    "APBDelayer: state not idle during reset")
    assert(transCycles  === 0.U,       "APBDelayer: transCycles not zero during reset")
    assert(delayCounter === 0.U,       "APBDelayer: delayCounter not zero during reset")
    assert(req_pwrite   === false.B,   "APBDelayer: req_pwrite not cleared during reset")
    assert(req_paddr    === 0.U,       "APBDelayer: req_paddr not cleared during reset")
    assert(req_pprot    === 0.U,       "APBDelayer: req_pprot not cleared during reset")
    assert(req_pwdata   === 0.U,       "APBDelayer: req_pwdata not cleared during reset")
    assert(req_pstrb    === 0.U,       "APBDelayer: req_pstrb not cleared during reset")
    assert(resp_prdata  === 0.U,       "APBDelayer: resp_prdata not cleared during reset")
    assert(resp_pslverr === false.B,   "APBDelayer: resp_pslverr not cleared during reset")
  }
}

class APBDelayerWrapper(implicit p: Parameters) extends LazyModule {
  val node = APBIdentityNode()

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      val delayer = Module(new APBDelayerChisel)
      delayer.io.clock := clock
      delayer.io.reset := reset
      delayer.io.in <> in
      out <> delayer.io.out
    }
  }
}

object APBDelayer {
  def apply()(implicit p: Parameters): APBNode = {
    val apbdelay = LazyModule(new APBDelayerWrapper)
    apbdelay.node
  }
}
