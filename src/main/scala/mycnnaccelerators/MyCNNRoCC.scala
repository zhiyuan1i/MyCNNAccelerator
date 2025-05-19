// filename: MyCNNRoCC.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._ // Make sure Mux, Fill, PopCount, etc. are available

import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.subsystem.MaxXLen
import freechips.rocketchip.tile._ // Pulls in HasCoreParameters, PRV, etc.
// M_XWR, M_XRD
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
// end


// Key for accessing the base AcceleratorConfig from the configuration system
case object MyCNNAcceleratorKey extends Field[AcceleratorConfig](DefaultAcceleratorConfig)

class MyCNNRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  val baseConfig = p(MyCNNAcceleratorKey)
  val systemMaxXLen = p(MaxXLen)
  val actualConfig = baseConfig.copy(xLen = systemMaxXLen)

  override lazy val module = new MyCNNRoCCModuleImp(this, actualConfig)
}

class MyCNNRoCCModuleImp(outer: MyCNNRoCC, val config: AcceleratorConfig)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer) { // LazyRoCCModuleImp mixes in HasCoreParameters, which brings in MemoryOpConstants, CSRs (for PRV), etc.

  val controller   = Module(new CNNController(config))
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

  // --- SimpleDMAController <-> RoCC Memory Interface (io.mem: HellaCacheIO) ---

  io.mem.req.valid := dma.io.mem_req.valid
  dma.io.mem_req.ready := io.mem.req.ready

  io.mem.req.bits.addr    := dma.io.mem_req.bits.addr
  io.mem.req.bits.cmd     := Mux(dma.io.mem_req.bits.isWrite, M_XWR, M_XRD) // M_XWR and M_XRD should be in scope now
  io.mem.req.bits.size    := dma.io.mem_req.bits.size
  io.mem.req.bits.signed  := false.B
  io.mem.req.bits.data    := dma.io.mem_req.bits.data
  io.mem.req.bits.phys    := true.B
  io.mem.req.bits.no_alloc:= false.B
  io.mem.req.bits.no_xcpt := false.B
  io.mem.req.bits.tag     := 0.U

  // Mask: HellaCacheReq requires a mask. DMA transfers full beats.
  // config.tileLinkBeatBytes is an Int
  io.mem.req.bits.mask    := Fill(config.tileLinkBeatBytes, 1.U(1.W))

  // dprv (privilege mode) and dv (data virtual address)
  // PRV should be in scope from HasCoreParameters (via CSRs)
  io.mem.req.bits.dprv    := PRV.M.U // Example: Machine mode. Adjust if necessary (e.g. PRV.S or PRV.U)
                                     // PRV.SZ is also available for width casting if needed, but 0.U works for U/S/M modes.
  io.mem.req.bits.dv      := false.B // Assuming physical addresses

  // idx (cache index for virtual address aliasing) - DMA uses physical addresses
  // usingVM, untagBits, pgIdxBits should be in scope from HasCoreParameters
  io.mem.req.bits.idx.foreach(_ := 0.U) // Assign to idx if it exists (is Some). Default to 0.U for physical addrs.

  io.mem.req.bits.no_resp := false.B // DMA controller expects a response

  // Connect RoCC's HellaCache memory response to DMA's memory response
  dma.io.mem_resp.valid     := io.mem.resp.valid
  dma.io.mem_resp.bits.data := io.mem.resp.bits.data

  // --- SimpleDMAController <-> Scratchpad Connections ---
  spad.io.ifm_write.en   := dma.io.spad_ifm_wen
  spad.io.ifm_write.addr := dma.io.spad_ifm_addr
  spad.io.ifm_write.data := dma.io.spad_ifm_data_in

  spad.io.kernel_write.en   := dma.io.spad_kernel_wen
  spad.io.kernel_write.addr := dma.io.spad_kernel_addr
  spad.io.kernel_write.data := dma.io.spad_kernel_data_in

  // Use .fire (Bool signal) not .fire() (Scala method)
  val latched_dma_is_write_to_main_mem = RegEnable(controller.io.dma_req_out.bits.is_write_to_main_mem, controller.io.dma_req_out.fire)
  spad.io.ofm_read.en   := dma.io.busy && latched_dma_is_write_to_main_mem

  spad.io.ofm_read.addr := dma.io.spad_ofm_addr
  dma.io.spad_ofm_data_out := spad.io.ofm_read.data

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