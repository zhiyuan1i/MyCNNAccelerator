// Suggested to be in a new file or at the top of ComputeUnitTest.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitSystemWrapperIO(val config: AcceleratorConfig) extends Bundle {
  // Control for ComputeUnit
  val start = Input(Bool())
  val done = Output(Bool()) // From ComputeUnit

  // Ports for Testbench to pre-load IFM buffer
  val ifm_buffer_write_en = Input(Bool())
  val ifm_buffer_write_addr = Input(UInt(config.ifmAddrWidth.W))
  val ifm_buffer_write_data = Input(SInt(config.dataWidth.W))

  // Ports for Testbench to pre-load Kernel buffer
  val kernel_buffer_write_en = Input(Bool())
  val kernel_buffer_write_addr = Input(UInt(config.kernelAddrWidth.W))
  val kernel_buffer_write_data = Input(SInt(config.dataWidth.W))

  // Ports for Testbench to read OFM buffer
  val ofm_buffer_read_addr = Input(UInt(config.ofmAddrWidth.W))
  val ofm_buffer_read_data = Output(SInt(config.dataWidth.W))
}

class ComputeUnitSystemWrapper(val config: AcceleratorConfig) extends Module {
  val io = IO(new ComputeUnitSystemWrapperIO(config))

  val ifm_buffer    = Module(new MinimalBuffer(config.ifmDepth, config.dataWidth))
  val kernel_buffer = Module(new MinimalBuffer(config.kernelDepth, config.dataWidth))
  val ofm_buffer    = Module(new MinimalBuffer(config.ofmDepth, config.dataWidth))
  val compute_unit  = Module(new ComputeUnit(config)) // Using your reverted 1-cycle address CU

  // Connect Testbench to IFM Buffer for writing
  ifm_buffer.io.write_en   := io.ifm_buffer_write_en
  ifm_buffer.io.write_addr := io.ifm_buffer_write_addr
  ifm_buffer.io.write_data := io.ifm_buffer_write_data
  // ComputeUnit reads from IFM Buffer
  compute_unit.io.ifm_read_data := ifm_buffer.io.read_data
  ifm_buffer.io.read_addr       := compute_unit.io.ifm_read_addr

  // Connect Testbench to Kernel Buffer for writing
  kernel_buffer.io.write_en   := io.kernel_buffer_write_en
  kernel_buffer.io.write_addr := io.kernel_buffer_write_addr
  kernel_buffer.io.write_data := io.kernel_buffer_write_data
  // ComputeUnit reads from Kernel Buffer
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data
  kernel_buffer.io.read_addr       := compute_unit.io.kernel_read_addr

  // ComputeUnit writes to OFM Buffer
  ofm_buffer.io.write_en   := compute_unit.io.ofm_write_en
  ofm_buffer.io.write_addr := compute_unit.io.ofm_write_addr
  ofm_buffer.io.write_data := compute_unit.io.ofm_write_data
  // Testbench reads from OFM Buffer
  ofm_buffer.io.read_addr    := io.ofm_buffer_read_addr
  io.ofm_buffer_read_data  := ofm_buffer.io.read_data

  // Connect ComputeUnit start/done signals
  compute_unit.io.start := io.start
  io.done               := compute_unit.io.done
}