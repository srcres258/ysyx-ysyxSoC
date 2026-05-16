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
  // 每 4-bit 控制一个数码管的低 4 段 (可扩展)
  val segReg = RegInit(0.U(32.W))

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

  // 数码管 — segReg 按 4-bit 分组映射到 8 个数码管
  for (i <- 0 until 8) {
    io.gpio.seg(i) := segReg(i * 4 + 3, i * 4)
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
