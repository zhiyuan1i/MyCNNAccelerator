// filename: MyCNNRoCC.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Field, Parameters}
// Import XLen (Field key) for looking up in Parameters
import freechips.rocketchip.subsystem.MaxXLen 
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket.constants.MemoryOpConstants

// Key for accessing the base AcceleratorConfig from the configuration system
case object MyCNNAcceleratorKey extends Field[AcceleratorConfig](DefaultAcceleratorConfig)

class MyCNNRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  // Fetch the base configuration provided by the system configuration
  val baseConfig = p(MyCNNAcceleratorKey)
  // Fetch the system's maximum xLen from the parameters using the imported MaxXLen key
  val systemMaxXLen = p(MaxXLen) // MaxXLen (uppercase) is the Field key

  // Create the final AcceleratorConfig instance to be used, copying base config and injecting the systemMaxXLen
  // Note: This uses the maximum XLen in the system. If a core-specific XLen is needed and available
  // via a different key (like the standard freechips.rocketchip.config.XLen), that might be more appropriate.
  // However, this change addresses the user's direct feedback regarding the import.
  val actualConfig = baseConfig.copy(xLen = systemMaxXLen) // xLen (lowercase) is the field in AcceleratorConfig

  // Pass the actualConfig (with xLen populated) to the module implementation
  override lazy val module = new MyCNNRoCCModuleImp(this, actualConfig)
}

class MyCNNRoCCModuleImp(outer: MyCNNRoCC, val config: AcceleratorConfig)(implicit p: Parameters)
    // LazyRoCCModuleImp already mixes in HasCoreParameters.
    // We are now making xLen explicitly available via the `config` object.
    extends LazyRoCCModuleImp(outer) {

  // Instantiate all sub-modules, passing the AcceleratorConfig (which now includes xLen)
  val controller   = Module(new CNNController(config)) // Pass the config object
  val dma          = Module(new SimpleDMAController(config))
  val spad         = Module(new Scratchpad(config))
  val compute_unit = Module(new ComputeUnit(config))

  // RoCC Interface Connections
  controller.io.cmd_in <> io.cmd
  io.resp.valid     := controller.io.cmd_resp_valid_out
  io.resp.bits.rd   := controller.io.cmd_resp_rd_out
  io.resp.bits.data := controller.io.cmd_resp_data_out
  io.busy           := controller.io.busy_out
  io.interrupt      := false.B

  // CNNController <-> SimpleDMAController
  dma.io.dma_req <> controller.io.dma_req_out
  controller.io.dma_busy_in  := dma.io.busy
  controller.io.dma_done_in  := dma.io.done
  controller.io.dma_error_in := dma.io.error

  // SimpleDMAController <-> RoCC Memory Interface
  dma.io.mem <> io.mem

  // SimpleDMAController <-> Scratchpad Connections
  val dma_is_writing_to_ifm = dma.io.spad_write_en && dma.io.dma_req.bits.spad_target_is_ifm && !dma.io.dma_req.bits.is_write_to_main_mem
  val dma_is_writing_to_kernel = dma.io.spad_write_en && dma.io.dma_req.bits.spad_target_is_kernel && !dma.io.dma_req.bits.is_write_to_main_mem

  spad.io.ifm_write.en   := dma_is_writing_to_ifm
  spad.io.ifm_write.addr := dma.io.spad_write_addr
  spad.io.ifm_write.data := dma.io.spad_write_data

  spad.io.kernel_write.en   := dma_is_writing_to_kernel
  spad.io.kernel_write.addr := dma.io.spad_write_addr
  spad.io.kernel_write.data := dma.io.spad_write_data

  spad.io.ofm_read.en   := dma.io.spad_read_en && dma.io.dma_req.bits.is_write_to_main_mem
  spad.io.ofm_read.addr := dma.io.spad_read_addr
  dma.io.spad_read_data := spad.io.ofm_read.data

  // ComputeUnit <-> Scratchpad Connections
  spad.io.ifm_read.en   := compute_unit.io.ifm_read_req.en
  spad.io.ifm_read.addr := compute_unit.io.ifm_read_req.addr
  compute_unit.io.ifm_read_req.data := spad.io.ifm_read.data

  spad.io.kernel_read.en   := compute_unit.io.kernel_read_req.en
  spad.io.kernel_read.addr := compute_unit.io.kernel_read_req.addr
  compute_unit.io.kernel_read_req.data := spad.io.kernel_read.data

  spad.io.ofm_write.en   := compute_unit.io.ofm_write_req.en
  spad.io.ofm_write.addr := compute_unit.io.ofm_write_req.addr
  spad.io.ofm_write.data := compute_unit.io.ofm_write_req.data

  // CNNController <-> ComputeUnit Connections
  compute_unit.io.start            := controller.io.compute_start_out
  compute_unit.io.kernel_dim       := controller.io.compute_kernel_dim_out
  controller.io.compute_busy_in    := compute_unit.io.busy
  controller.io.compute_done_in    := compute_unit.io.done
  controller.io.compute_error_in   := compute_unit.io.error
}
