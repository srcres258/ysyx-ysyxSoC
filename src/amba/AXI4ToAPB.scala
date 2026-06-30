package ysyx

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.amba._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.amba.apb._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

case class AXI4ToAPBNode()(implicit valName: ValName) extends MixedAdapterNode(AXI4Imp, APBImp)(
  dFn = { mp =>
    APBMasterPortParameters(
      masters = mp.masters.map { m => APBMasterParameters(name = m.name, nodePath = m.nodePath) },
      requestFields = mp.requestFields.filter(!_.isInstanceOf[AMBAProtField]),
      responseKeys  = mp.responseKeys
    )
  },
  uFn = { sp =>
    val beatBytes = 4
    AXI4SlavePortParameters(
    slaves = sp.slaves.map { s =>
      val maxXfer = TransferSizes(1, beatBytes)
      require(beatBytes == 4) // only support 8-byte data AXI
      AXI4SlaveParameters(
        address       = s.address,
        resources     = s.resources,
        regionType    = s.regionType,
        executable    = s.executable,
        nodePath      = s.nodePath,
        supportsWrite = if (s.supportsWrite) TransferSizes(1, beatBytes) else TransferSizes.none,
        supportsRead  = if (s.supportsRead)  TransferSizes(1, beatBytes) else TransferSizes.none,
        interleavedId = Some(0))}, // never interleaves D beats
    beatBytes = beatBytes,
    responseFields = sp.responseFields,
    requestKeys    = sp.requestKeys.filter(_ != AMBAProt))
  }
)

class AXI4ToAPB(val aFlow: Boolean = true)(implicit p: Parameters) extends LazyModule {
  val node = AXI4ToAPBNode()

  lazy val module = new LazyModuleImp(this) {
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      val (ar, r, aw, w, b) = (in.ar, in.r, in.aw, in.w, in.b)

      val s_idle :: s_inflight :: s_wait_rready_bready :: Nil = Enum(3)
      val state = RegInit(s_idle)
      val accept_read = (state === s_idle) && ar.valid
      val accept_write = !accept_read && (state === s_idle) && aw.valid && w.valid
      val is_write = accept_write holdUnless (state === s_idle)
      switch (state) {
        is (s_idle)     { state := Mux(ar.valid || (aw.valid && w.valid), s_inflight, s_idle) }
        is (s_inflight) { state := Mux(out.pready, Mux(r.fire || b.fire, s_idle, s_wait_rready_bready), s_inflight) }
        is (s_wait_rready_bready) { state := Mux(r.fire || b.fire, s_idle, s_wait_rready_bready) }
      }

      // burst is not supported
      assert(!(ar.valid && ar.bits.len =/= 0.U))
      assert(!(aw.valid && aw.bits.len =/= 0.U))
      // size > 4 is not supported
      assert(!(ar.valid && ar.bits.size > "b10".U))
      assert(!(aw.valid && aw.bits.size > "b10".U))

      val rid_reg    = RegEnable(ar.bits.id, accept_read)
      val bid_reg    = RegEnable(aw.bits.id, accept_write)
      val araddr_reg = ar.bits.addr holdUnless accept_read
      val awaddr_reg = aw.bits.addr holdUnless accept_write
      val wdata_reg  =  w.bits.data holdUnless accept_write
      val wstrb_reg  =  w.bits.strb holdUnless accept_write

      out.psel    := (accept_read || accept_write) || out.penable
      out.penable := state === s_inflight
      out.pwrite  := is_write
      out.paddr   := Mux(is_write, awaddr_reg, araddr_reg)
      out.pprot   := APBParameters.PROT_DEFAULT
      // CPU now sends byte-aligned data/strobe on the correct AXI4 byte lanes.
      //   - PSRAM/SDRAM: target uses multi-byte APB with pstrb; pass through as-is.
      //   - UART/GPIO/etc (peripheral): target only reads pwdata[7:0]; shift right
      //     to bring the active byte back to lane 0, and force pstrb to 0b0001.
      val is_psram_w = awaddr_reg >= 0x80000000L.U && awaddr_reg < 0x80400000L.U
      val is_sdram_w = awaddr_reg >= 0xa0000000L.U && awaddr_reg < 0xa8000000L.U
      val is_mem_w    = is_psram_w || is_sdram_w
      val is_periph_w = !is_psram_w && !is_sdram_w
      val wshift = Cat(awaddr_reg(1,0), 0.U(3.W))
      out.pwdata := Mux(
        is_write,
        Mux(
          // PSRAM/SDRAM: CPU pre-aligned, pass through
          is_mem_w,
          wdata_reg,                        
          // peripheral: shift down to lane 0
          Mux(is_periph_w, wdata_reg >> wshift, 0.U)
        ),
        0.U
      )
      out.pstrb := Mux(
        is_write,
        Mux(
          // PSRAM/SDRAM: pass through
          is_mem_w,
          wstrb_reg,                    
          // peripheral: lane 0 only
          Mux(is_periph_w, 0b0001.U, 0.U)
        ),              
        0.U
      )

      ar.ready := accept_read
      w.ready  := accept_write
      aw.ready := accept_write

      val resp = Mux(out.pslverr, AXI4Parameters.RESP_SLVERR, AXI4Parameters.RESP_OKAY)
      val resp_hold = resp holdUnless (state === s_inflight)
      r.valid  := !is_write && (((state === s_inflight) && out.pready) || (state === s_wait_rready_bready))
      val rdata_raw = out.prdata holdUnless (state === s_inflight)
      // CPU MEMUnit right-shifts rdata by addr*8 before extracting
      // sub-word lanes (expects data in the AXI4-standard byte lane).
      //   - PSRAM/SDRAM: word-aligned targets, data already at natural byte
      //     positions → pass through.
      //   - FLASH/UART/etc: APB targets return the byte in prdata[7:0]
      //     regardless of address offset → shift LEFT by addr*8 to the
      //     correct AXI4 byte lane.
      val is_psram     = araddr_reg >= 0x80000000L.U && araddr_reg < 0x80400000L.U
      val is_sdram     = araddr_reg >= 0xa0000000L.U && araddr_reg < 0xa8000000L.U
      val is_periph    = !is_psram && !is_sdram
      val rshift       = Cat(araddr_reg(1,0), 0.U(3.W))
      r.bits.data := Fill(
        2,
        Mux(
          // FLASH/UART: shift left to byte lane
          is_periph,
          rdata_raw << rshift,   
          // PSRAM/SDRAM: pass through
          Mux(is_psram || is_sdram, rdata_raw, rdata_raw)
        )
      )
      r.bits.id   := rid_reg
      r.bits.resp := resp_hold
      r.bits.last := true.B

      b.valid  := is_write && (((state === s_inflight) && out.pready) || (state === s_wait_rready_bready))
      b.bits.resp := resp_hold
      b.bits.id   := bid_reg
    }
  }
}

object AXI4ToAPB {
  def apply(aFlow: Boolean = true)(implicit p: Parameters) = {
    val axi42apb = LazyModule(new AXI4ToAPB(aFlow))
    axi42apb.node
  }
}
