package ysyx

import chisel3._
import chisel3.util._

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class GPIOIO extends Bundle {
  val out = Output(UInt(16.W))
  val in = Input(UInt(16.W))
  val seg = Output(Vec(8, UInt(8.W)))
}

class GPIOCtrlIO extends Bundle {
  val clock = Input(Clock())
  val reset = Input(Reset())
  val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
  val gpio = new GPIOIO
}

class gpio_top_apb extends BlackBox {
  val io = IO(new GPIOCtrlIO)
}

class gpioChisel extends Module {
  val io = IO(new GPIOCtrlIO)

  // ========================
  // 内部寄存器
  // ========================

  // LED 输出寄存器 (偏移 0x0) — 16-bit, 复位值 0
  val ledReg = RegInit(0.U(16.W))

  // 7段数码管段选寄存器 (偏移 0x8) — 32-bit, 复位值 0
  // 每 4-bit 控制一个数码管的显示值 (0-F)
  val segReg = RegInit(0.U(32.W))

  // ========================
  // BCD (4-bit hex) 到 7 段 (8-bit) 译码表
  // ========================
  // NVBoard 段选位序: [7:A, 6:B, 5:C, 4:D, 3:E, 2:F, 1:G, 0:DP]
  // 高电平有效: 1 = 段亮
  val segTable = VecInit(Seq(
    "b11111100".U(8.W),  // 0: ABCDEF
    "b01100000".U(8.W),  // 1: BC
    "b11011010".U(8.W),  // 2: ABDEG
    "b11110010".U(8.W),  // 3: ABCDG
    "b01100110".U(8.W),  // 4: BCGF
    "b10110110".U(8.W),  // 5: ACDGF
    "b10111110".U(8.W),  // 6: ACDEFG
    "b11100000".U(8.W),  // 7: ABC
    "b11111110".U(8.W),  // 8: ABCDEFG
    "b11110110".U(8.W),  // 9: ABCDFG
    "b11101110".U(8.W),  // A: ABCEFG
    "b00111110".U(8.W),  // b: CDEFG
    "b10011100".U(8.W),  // C: ADEF
    "b01111010".U(8.W),  // d: BCDEG
    "b10011110".U(8.W),  // E: ADEFG
    "b10001110".U(8.W)   // F: AEFG
  ))

  // ========================
  // APB 地址译码 — 取低 4 位作偏移
  // ========================
  val offset = io.in.paddr(3, 0)

  // 写使能信号
  val writeActive = io.in.pwrite && io.in.psel && io.in.penable
  // 读使能信号
  val readActive  = !io.in.pwrite && io.in.psel && io.in.penable

  // ========================
  // APB 写逻辑
  // ========================
  when (writeActive) {
    switch (offset) {
      is (0x0.U) { ledReg := io.in.pwdata(15, 0) }
      is (0x8.U) { segReg := io.in.pwdata }
    }
  }

  // ========================
  // APB 读逻辑
  // ========================
  io.in.prdata := 0.U
  when (readActive) {
    switch (offset) {
      is (0x0.U) { io.in.prdata := Cat(0.U(16.W), ledReg) }
      is (0x4.U) { io.in.prdata := Cat(0.U(16.W), io.gpio.in) }
      is (0x8.U) { io.in.prdata := segReg }
    }
  }

  // ========================
  // APB 握手信号 — 组合逻辑，当拍响应
  // ========================
  io.in.pready  := io.in.psel && io.in.penable
  io.in.pslverr := false.B

  // ========================
  // GPIO 物理输出
  // ========================
  // LED — 由寄存器直接驱动
  io.gpio.out := ledReg

  // 数码管 — segReg 按 4-bit 分组, 经译码表转换为 8-bit 段码
  for (i <- 0 until 8) {
    val hexVal = segReg(i * 4 + 3, i * 4)
    io.gpio.seg(i) := segTable(hexVal)
  }
}

class APBGPIO(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val gpio_bundle = IO(new GPIOIO)

    val mgpio = Module(new gpioChisel)
    mgpio.io.clock := clock
    mgpio.io.reset := reset
    mgpio.io.in <> in
    gpio_bundle <> mgpio.io.gpio
  }
}
