package ysyx

import chisel3._
import org.chipsalliance.cde.config.{Parameters, Config}
import freechips.rocketchip.system._
import freechips.rocketchip.diplomacy.LazyModule

object Config {
  def hasChipLink: Boolean = false
  def sdramUseAXI: Boolean = false

  // SDRAM extension control
  //   sdramBitExt:  enable 16→32 bit bit-extension (2颗粒 per channel)
  //   sdramWordExt: enable word-extension (doubles channel count, 4颗粒 total)
  //   When sdramBitExt=false, sdramWordExt is meaningless and ignored.
  def sdramBitExt: Boolean = true   // default: enabled
  def sdramWordExt: Boolean = true  // default: enabled

  // Derived: total SDRAM颗粒 count
  def sdramParticleCount: Int =
    (if (sdramBitExt) 2 else 1) * (if (sdramWordExt) 2 else 1)

  // Derived: total capacity in bytes
  def sdramCapacityBytes: Long = 32L * 1024 * 1024 * sdramParticleCount
  // 1颗粒: 32MB, 2颗粒: 64MB, 4颗粒: 128MB

  // Derived: address space size (must be power-of-2, >= capacity)
  def sdramAddressSpaceSize: Long = {
    val cap = sdramCapacityBytes
    var sz = 1L << 25  // 32MB minimum
    while (sz < cap) sz <<= 1
    sz
  }

  require(!sdramWordExt || sdramBitExt,
    "SDRAM word extension requires bit extension to be enabled")
}

class ysyxSoCTop extends Module {
  implicit val config: Parameters = new Config(new Edge32BitConfig ++ new DefaultRV32Config)

  val io = IO(new Bundle { })
  val dut = LazyModule(new ysyxSoCFull)
  val mdut = Module(dut.module)
  mdut.dontTouchPorts()
  mdut.externalPins := DontCare
}

object Elaborate extends App {
  val firtoolOptions = Array("--disable-annotation-unknown")
  circt.stage.ChiselStage.emitSystemVerilogFile(new ysyxSoCTop, args, firtoolOptions)
}
