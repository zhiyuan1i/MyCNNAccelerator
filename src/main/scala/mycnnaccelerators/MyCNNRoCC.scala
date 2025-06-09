// filename: MyCNNRoCC.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy._
// import freechips.rocketchip.tilelink.TLIdentityNode // Not directly needed here with override

import CNNAcceleratorISA._ // Import ISA definitions

// Key for configuration
case object MyCNNAcceleratorKey extends Field[AcceleratorConfig](DefaultAcceleratorConfig)

class MyCNNRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  val baseConfig = p(MyCNNAcceleratorKey) // Get base config from Parameters

  // Instantiate the DMA LazyModule.
  // The AcceleratorConfig might need xLen, which is known in the ModuleImp.
  // SimpleDMAEngine takes accConfig.
  // val dma_engine_lazy = LazyModule(new SimpleDMAEngine(baseConfig))
  val dma_engine_lazy = LazyModule(new PipelinedDMAEngine(baseConfig))

  // This RoCC will use the dma_engine_lazy's TileLink node as its primary memory master interface.
  // Override atlNode from CanHavePTW (mixed into LazyRoCC) or a similar node if defined by LazyRoCC.
  // This effectively makes the DMA the RoCC's connection to the memory system via TileLink.
  override val atlNode = dma_engine_lazy.node

  // The actual config with correct xLen is created in MyCNNRoCCModuleImp
  override lazy val module = new MyCNNRoCCModuleImp(this, baseConfig)
}