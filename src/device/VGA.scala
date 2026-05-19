package ysyx

import chisel3._
import chisel3.util._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class VGAIO extends Bundle {
  val r = Output(UInt(8.W))
  val g = Output(UInt(8.W))
  val b = Output(UInt(8.W))
  val hsync = Output(Bool())
  val vsync = Output(Bool())
  val valid = Output(Bool())
}

class VGACtrlIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Bool())
  val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val vga = new VGAIO
}

class vga_top_apb extends BlackBox {
  val io = IO(new VGACtrlIO)
}

class vgaChisel extends Module {
  val io = IO(new VGACtrlIO)

  // ============================================================
  // VGA 时序参数 (640x480 @ 60Hz)
  // ============================================================
  val H_SYNC   = 96
  val H_BACK   = 48
  val H_ACTIVE = 640
  val H_FRONT  = 16
  val H_TOTAL  = 800

  val V_SYNC   = 2
  val V_BACK   = 33
  val V_ACTIVE = 480
  val V_FRONT  = 10
  val V_TOTAL  = 525

  val hCnt = RegInit(0.U(10.W))
  val vCnt = RegInit(0.U(10.W))

  when (io.reset) {
    hCnt := 0.U
    vCnt := 0.U
  }.otherwise {
    when (hCnt === (H_TOTAL - 1).U) {
      hCnt := 0.U
      when (vCnt === (V_TOTAL - 1).U) {
        vCnt := 0.U
      }.otherwise {
        vCnt := vCnt + 1.U
      }
    }.otherwise {
      hCnt := hCnt + 1.U
    }
  }

  val hSync = hCnt >= H_SYNC.U
  val vSync = vCnt >= V_SYNC.U

  val hActiveStart = (H_SYNC + H_BACK).U
  val hActiveEnd   = (H_SYNC + H_BACK + H_ACTIVE).U
  val vActiveStart = (V_SYNC + V_BACK).U
  val vActiveEnd   = (V_SYNC + V_BACK + V_ACTIVE).U

  val hValid = hCnt >= hActiveStart && hCnt < hActiveEnd
  val vValid = vCnt >= vActiveStart && vCnt < vActiveEnd
  val pixelValid = hValid && vValid

  val pixelX = hCnt - hActiveStart
  val pixelY = vCnt - vActiveStart
  val pixelAddr = pixelY * 640.U + pixelX

  val fbDepth = 640 * 480
  val framebuffer = Mem(fbDepth, UInt(32.W))

  // APB 帧缓冲写入: VGA_BASE=0x21000000 未对齐, APB扇出器无法正确截断高位.
  // 用取模运算求地址偏移 (FIRRTL 无法优化掉取模).
  val VGA_BASE = 0x21000000L.U
  val VGA_SIZE = 0x200000L.U  // 2MB
  val apbOffset = (io.in.paddr - VGA_BASE) % VGA_SIZE
  val apbWrite  = io.in.pwrite && io.in.psel && io.in.penable

  when (apbWrite && !io.reset && apbOffset >= 8.U) {
    val fbIndex = (apbOffset - 8.U)(20, 2)
    framebuffer.write(fbIndex, io.in.pwdata)
  }

  val fbRdData = framebuffer(pixelAddr)
  io.vga.r     := fbRdData(23, 16)
  io.vga.g     := fbRdData(15, 8)
  io.vga.b     := fbRdData(7, 0)
  io.vga.hsync := hSync
  io.vga.vsync := vSync
  io.vga.valid := pixelValid

  when (!pixelValid) {
    io.vga.r := 0.U
    io.vga.g := 0.U
    io.vga.b := 0.U
  }

  when (!io.in.pwrite && io.in.psel && io.in.penable) {
    when (apbOffset === 0.U) {
      io.in.prdata := Cat(640.U(16.W), 480.U(16.W))
    }.elsewhen (apbOffset >= 8.U) {
      val fbIndex = (apbOffset - 8.U)(20, 2)
      io.in.prdata := framebuffer(fbIndex)
    }.otherwise {
      io.in.prdata := 0.U
    }
  }.otherwise {
    io.in.prdata := 0.U
  }

  io.in.pready  := io.in.psel && io.in.penable
  io.in.pslverr := false.B
}

class APBVGA(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val vga_bundle = IO(new VGAIO)

    val mvga = Module(new vgaChisel)
    mvga.io.clock := clock
    mvga.io.reset := reset
    mvga.io.in <> in
    vga_bundle <> mvga.io.vga
  }
}