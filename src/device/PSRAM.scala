package ysyx

import chisel3._
import chisel3.util._
import chisel3.experimental.Analog

import freechips.rocketchip.amba.apb._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._

class QSPIIO extends Bundle {
  val sck = Output(Bool())
  val ce_n = Output(Bool())
  val dio = Analog(4.W)
}

class psram_top_apb extends BlackBox {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Reset())
    val in = Flipped(new APBBundle(APBBundleParameters(addrBits = 32, dataBits = 32)))
    val qspi = new QSPIIO
  })
}

class psram extends BlackBox {
  val io = IO(Flipped(new QSPIIO))
}

class psramChisel extends RawModule {
  val io = IO(Flipped(new QSPIIO))
  val di = TriStateInBuf(io.dio, 0.U, false.B) // change this if you need
}

class PSRAMMemBundle extends Bundle {
  val readAddr = Input(UInt(24.W))
  val readEnable = Input(Bool())
  val readData = Output(UInt(8.W))

  val writeAddr = Input(UInt(24.W))
  val writeEnable = Input(Bool())
  val writeData = Input(UInt(8.W))
}

class PSRAMMemImplDPIC extends BlackBox with HasBlackBoxInline {
  val io = IO(new PSRAMMemBundle {
    val clock = Input(Bool())
    val reset = Input(Bool())
  })

  setInline(
    "PSRAMMemImplDPIC.v",
    """module PSRAMMemImplDPIC(
      |    input clock,
      |    input reset,
      |
      |    input [23:0] readAddr,
      |    input readEnable,
      |    output reg [7:0] readData /* verilator public_flat */,
      |
      |    input [23:0] writeAddr,
      |    input writeEnable,
      |    input [7:0] writeData
      |);
      |    import "DPI-C" function void psram_read(input [23:0] raddr, output byte rdata);
      |    import "DPI-C" function void psram_write(input [23:0] waddr, input byte wdata);
      |
      |    always @(posedge clock) begin
      |        if (readEnable) psram_read(readAddr, readData);
      |        else readData = 0;
      |
      |        if (writeEnable) psram_write(writeAddr, writeData);
      |    end
      |endmodule
    """.stripMargin
  )
}

class PSRAMMemImplChisel extends Module {
  val io = IO(new PSRAMMemBundle)

  val mem = SyncReadMem(1 << 24, UInt(8.W))
  io.readData := Mux(io.readEnable, mem.read(io.readAddr), 0.U)
  when(io.writeEnable) {
    mem.write(io.writeAddr, io.writeData)
  }
}

class PSRAMMem(useDPIC: Boolean) extends Module {
  val io = IO(new PSRAMMemBundle)

  if (useDPIC) {
    val impl = Module(new PSRAMMemImplDPIC)
    impl.io.clock := clock.asBool
    impl.io.reset := reset.asBool
    impl.io.readAddr := io.readAddr
    impl.io.readEnable := io.readEnable
    io.readData := impl.io.readData
    impl.io.writeAddr := io.writeAddr
    impl.io.writeEnable := io.writeEnable
    impl.io.writeData := io.writeData
  } else {
    val impl = Module(new PSRAMMemImplChisel)
    impl.io <> io
  }
}

class APBPSRAM(address: Seq[AddressSet])(implicit p: Parameters) extends LazyModule {
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
    val qspi_bundle = IO(new QSPIIO)

    val mpsram = Module(new psram_top_apb)
    mpsram.io.clock := clock
    mpsram.io.reset := reset
    mpsram.io.in <> in
    qspi_bundle <> mpsram.io.qspi
  }
}
