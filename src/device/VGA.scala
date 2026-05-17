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
  val H_BACK   = 48    // 144 - 96
  val H_ACTIVE = 640
  val H_FRONT  = 16    // 800 - 784
  val H_TOTAL  = 800

  val V_SYNC   = 2
  val V_BACK   = 33    // 35 - 2
  val V_ACTIVE = 480
  val V_FRONT  = 10    // 525 - 515
  val V_TOTAL  = 525

  // ============================================================
  // 水平和垂直计数器
  // ============================================================
  val hCnt = RegInit(0.U(10.W))   // 0 ~ 799
  val vCnt = RegInit(0.U(10.W))   // 0 ~ 524

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

  // ============================================================
  // 同步信号生成 (active-low: 同步脉冲期间为低)
  // ============================================================
  val hSync = hCnt >= H_SYNC.U          // 0~95 低, 96~799 高
  val vSync = vCnt >= V_SYNC.U          // 0~1 低, 2~524 高

  // ============================================================
  // 有效区域判断
  // ============================================================
  val hActiveStart = (H_SYNC + H_BACK).U            // 144
  val hActiveEnd   = (H_SYNC + H_BACK + H_ACTIVE).U // 784
  val vActiveStart = (V_SYNC + V_BACK).U            // 35
  val vActiveEnd   = (V_SYNC + V_BACK + V_ACTIVE).U // 515

  val hValid = hCnt >= hActiveStart && hCnt < hActiveEnd
  val vValid = vCnt >= vActiveStart && vCnt < vActiveEnd
  val pixelValid = hValid && vValid

  // 当前像素坐标 (在有效区域内)
  val pixelX = hCnt - hActiveStart   // 0 ~ 639
  val pixelY = vCnt - vActiveStart   // 0 ~ 479
  val pixelAddr = pixelY * 640.U + pixelX  // 0 ~ 307199

  // ============================================================
  // 帧缓冲存储器 (640x480 像素, 每像素 32-bit = RGBX)
  // 使用 Mem (组合逻辑读, 无读延迟)
  // ============================================================
  val fbDepth = 640 * 480   // = 307200
  val framebuffer = Mem(fbDepth, UInt(32.W))

  // 来自 APB 的帧缓冲写入信号
  val apbOffset = io.in.paddr(20, 0)            // 21 位覆盖 2MB 空间
  val apbWrite  = io.in.pwrite && io.in.psel && io.in.penable

  // 帧缓冲写端口 (来自 APB)
  when (apbWrite && !io.reset && apbOffset >= 8.U) {
    val fbIndex = (apbOffset - 8.U)(20, 2)      // (offset - 8) / 4
    framebuffer.write(fbIndex, io.in.pwdata)
  }

  // ============================================================
  // VGA 输出 (组合逻辑读帧缓冲, 无延迟)
  // ============================================================
  val fbRdData = framebuffer(pixelAddr)
  io.vga.r     := fbRdData(23, 16)  // 红色
  io.vga.g     := fbRdData(15, 8)   // 绿色
  io.vga.b     := fbRdData(7, 0)    // 蓝色
  io.vga.hsync := hSync
  io.vga.vsync := vSync
  io.vga.valid := pixelValid

  // 非 active 期间输出黑色
  when (!pixelValid) {
    io.vga.r := 0.U
    io.vga.g := 0.U
    io.vga.b := 0.U
  }

  // ============================================================
  // APB 读逻辑
  // ============================================================
  when (!io.in.pwrite && io.in.psel && io.in.penable) {
    when (apbOffset === 0.U) {
      // GPU 配置: width[31:16] | height[15:0] = 640, 480
      io.in.prdata := Cat(640.U(16.W), 480.U(16.W))
    }.elsewhen (apbOffset >= 8.U) {
      // 帧缓冲读取
      val fbIndex = (apbOffset - 8.U)(20, 2)
      io.in.prdata := framebuffer(fbIndex)
    }.otherwise {
      // Sync 寄存器读 (偏移 4) 或未定义区域
      io.in.prdata := 0.U
    }
  }.otherwise {
    io.in.prdata := 0.U
  }

  // ============================================================
  // APB 握手信号 (单周期响应, 无等待状态)
  // ============================================================
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
