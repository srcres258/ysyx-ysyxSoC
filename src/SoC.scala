package ysyx

import chisel3._
import chisel3.util._

import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.subsystem._
import freechips.rocketchip.util._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.amba.apb._
import freechips.rocketchip.system.SimAXIMem

object AXI4SlaveNodeGenerator {
  def apply(params: Option[MasterPortParams], address: Seq[AddressSet])(implicit valName: ValName) =
    AXI4SlaveNode(params.map(p => AXI4SlavePortParameters(
        slaves = Seq(AXI4SlaveParameters(
          address       = address,
          executable    = p.executable,
          supportsWrite = TransferSizes(1, p.maxXferBytes),
          supportsRead  = TransferSizes(1, p.maxXferBytes))),
        beatBytes = p.beatBytes
      )).toSeq)
}

class ysyxSoCASIC(implicit p: Parameters) extends LazyModule {
  val xbar = AXI4Xbar()
  val xbar2 = AXI4Xbar()
  val apbxbar = LazyModule(new APBFanout).node
  val cpu = LazyModule(new CPU(idBits = ChipLinkParam.idBits))
  val chipMaster = if (Config.hasChipLink) Some(LazyModule(new ChipLinkMaster)) else None
  val chiplinkNode = if (Config.hasChipLink) Some(AXI4SlaveNodeGenerator(p(ExtBus), ChipLinkParam.allSpace)) else None

  val luart = LazyModule(new APBUart16550(AddressSet.misaligned(0x10000000, 0x1000)))
  val lgpio = LazyModule(new APBGPIO(AddressSet.misaligned(0x10002000, 0x10)))
  val lkeyboard = LazyModule(new APBKeyboard(AddressSet.misaligned(0x10011000, 0x8)))
  val lvga = LazyModule(new APBVGA(AddressSet.misaligned(0x21000000, 0x200000)))
  val lspi  = LazyModule(new APBSPI(
    AddressSet.misaligned(0x10001000, 0x1000) ++    // SPI controller
    AddressSet.misaligned(0x30000000, 0x10000000)   // XIP flash
  ))
  val lpsram = LazyModule(new APBPSRAM(AddressSet.misaligned(0x80000000L, 0x400000)))
  val lmrom = LazyModule(new AXI4MROM(AddressSet.misaligned(0x20000000, 0x1000)))
  val sramNode = AXI4RAM(AddressSet.misaligned(0x0f000000, 0x2000).head, false, true, 4, None, Nil, false)

  // SDRAM address space — computed from Config
  val sdramAddrSets = SDRAMAddr.addressSets  // Seq[AddressSet], 1 or 2 entries
  val sdramHasWordExt = Config.sdramBitExt && Config.sdramWordExt

  // Channel 0 (always present)
  val lsdram_apb = if (!Config.sdramUseAXI)
    Some(LazyModule(new APBSDRAM (Seq(sdramAddrSets(0))))) else None
  val lsdram_axi = if ( Config.sdramUseAXI)
    Some(LazyModule(new AXI4SDRAM(Seq(sdramAddrSets(0))))) else None

  // Channel 1 (only when word extension is active)
  val lsdram_apb_ch1 = if (!Config.sdramUseAXI && sdramHasWordExt)
    Some(LazyModule(new APBSDRAM (Seq(sdramAddrSets(1))))) else None
  val lsdram_axi_ch1 = if ( Config.sdramUseAXI && sdramHasWordExt)
    Some(LazyModule(new AXI4SDRAM(Seq(sdramAddrSets(1))))) else None

  List(lspi.node, luart.node, lpsram.node, lgpio.node, lkeyboard.node, lvga.node).map(_ := apbxbar)
  List(apbxbar := APBDelayer() := AXI4ToAPB() := AXI4Buffer(), lmrom.node, sramNode).map(_ := xbar2)
  xbar2 := AXI4UserYanker(Some(1)) := AXI4Fragmenter() := xbar
  if (Config.sdramUseAXI) {
    lsdram_axi.get.node := ysyx.AXI4Delayer() := xbar
    if (lsdram_axi_ch1.isDefined) lsdram_axi_ch1.get.node := ysyx.AXI4Delayer() := xbar
  } else {
    lsdram_apb.get.node := apbxbar
    if (lsdram_apb_ch1.isDefined) lsdram_apb_ch1.get.node := apbxbar
  }
  if (Config.hasChipLink) chiplinkNode.get := xbar
  xbar := cpu.masterNode

  override lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with DontTouch {
    // generate delayed reset for cpu, since chiplink should finish reset
    // to initialize some async modules before accept any requests from cpu
    cpu.module.reset := SynchronizerShiftReg(reset.asBool, 10) || reset.asBool

    val fpga_io = if (Config.hasChipLink) Some(IO(chiselTypeOf(chipMaster.get.module.fpga_io))) else None
    if (Config.hasChipLink) {
      // connect chiplink slave interface to crossbar
      (chipMaster.get.slave zip chiplinkNode.get.in) foreach { case (io, (bundle, _)) => io <> bundle }

      // connect chiplink dma interface to cpu
      cpu.module.slave <> chipMaster.get.master_mem(0)

      // expose chiplink fpga I/O interface as ports
      fpga_io.get <> chipMaster.get.module.fpga_io
    } else {
      cpu.module.slave := DontCare
    }

    // connect interrupt signal to cpu
    val intr_from_chipSlave = IO(Input(Bool()))
    cpu.module.interrupt := intr_from_chipSlave

    // --- SDRAM port(s) ---
    val sdramBundle0 = if (Config.sdramUseAXI) lsdram_axi.get.module.sdram_bundle
                       else                    lsdram_apb.get.module.sdram_bundle

    // expose slave I/O interface as ports
    val spi = IO(chiselTypeOf(lspi.module.spi_bundle))
    val uart = IO(chiselTypeOf(luart.module.uart))
    val psram = IO(chiselTypeOf(lpsram.module.qspi_bundle))
    val sdram = IO(chiselTypeOf(sdramBundle0))
    val gpio = IO(chiselTypeOf(lgpio.module.gpio_bundle))
    val ps2 = IO(chiselTypeOf(lkeyboard.module.ps2_bundle))
    val vga = IO(chiselTypeOf(lvga.module.vga_bundle))
    uart <> luart.module.uart
    spi <> lspi.module.spi_bundle
    psram <> lpsram.module.qspi_bundle
    sdram <> sdramBundle0
    gpio <> lgpio.module.gpio_bundle
    ps2 <> lkeyboard.module.ps2_bundle
    vga <> lvga.module.vga_bundle

    // Channel 1 SDRAM port (only when word extension is active)
    // Use Option pattern so the field always exists in the type — similar to fpga_io.
    val sdram1 = if (Config.sdramBitExt && Config.sdramWordExt) {
      val sdramBundle1 = if (Config.sdramUseAXI) lsdram_axi_ch1.get.module.sdram_bundle
                         else                    lsdram_apb_ch1.get.module.sdram_bundle
      val io1 = IO(chiselTypeOf(sdramBundle1))
      io1 <> sdramBundle1
      Some(io1)
    } else None
  }
}

class ysyxSoCFPGA(implicit p: Parameters) extends ChipLinkSlave


class ysyxSoCFull(implicit p: Parameters) extends LazyModule {
  val asic = LazyModule(new ysyxSoCASIC)
  ElaborationArtefacts.add("graphml", graphML)

  override lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with DontTouch {
    val masic = asic.module

    if (Config.hasChipLink) {
      val fpga = LazyModule(new ysyxSoCFPGA)
      val mfpga = Module(fpga.module)
      masic.dontTouchPorts()

      masic.fpga_io.get.b2c <> mfpga.fpga_io.c2b
      mfpga.fpga_io.b2c <> masic.fpga_io.get.c2b

      (fpga.master_mem zip fpga.axi4MasterMemNode.in).map { case (io, (_, edge)) =>
        val mem = LazyModule(new SimAXIMem(edge,
          base = ChipLinkParam.mem.base, size = ChipLinkParam.mem.mask + 1))
        Module(mem.module)
        mem.io_axi4.head <> io
      }

      fpga.master_mmio.map(_ := DontCare)
      fpga.slave.map(_ := DontCare)
    }

    masic.intr_from_chipSlave := false.B

    val flash = Module(new flash)
    flash.io <> masic.spi
    flash.io.ss := masic.spi.ss(0)
    val bitrev = Module(new bitrev)
    bitrev.io <> masic.spi
    bitrev.io.ss := masic.spi.ss(7)
    masic.spi.miso := List(bitrev.io, flash.io).map(_.miso).reduce(_&&_)

    val psram = Module(new psram)
    psram.io <> masic.psram
    // SDRAM behavior model — instantiate 1, 2, or 4颗粒 based on Config
    if (Config.sdramBitExt && Config.sdramWordExt) {
      // 4颗粒: two bit-extension pairs in dual-channel word-extension
      val sdram0 = Module(new sdramChisel(laneCount = 2, baseOffsetBytes = 0L))
      val sdram1 = Module(new sdramChisel(laneCount = 2, baseOffsetBytes = 0x4000000L))
      sdram0.io <> masic.sdram
      sdram1.io <> masic.sdram1.get
    } else if (Config.sdramBitExt) {
      // 2颗粒: single-channel bit-extension (32-bit data)
      val sdram = Module(new sdramChisel(laneCount = 2, baseOffsetBytes = 0L))
      sdram.io <> masic.sdram
    } else {
      // 1颗粒: no extension (16-bit data)
      val sdram = Module(new sdramChisel(laneCount = 1, baseOffsetBytes = 0L))
      sdram.io <> masic.sdram
    }

    val externalPins = IO(new Bundle{
      val gpio = chiselTypeOf(masic.gpio)
      val ps2 = chiselTypeOf(masic.ps2)
      val vga = chiselTypeOf(masic.vga)
      val uart = chiselTypeOf(masic.uart)
    })
    externalPins.gpio <> masic.gpio
    externalPins.ps2 <> masic.ps2
    externalPins.vga <> masic.vga
    externalPins.uart <> masic.uart
  }
}
