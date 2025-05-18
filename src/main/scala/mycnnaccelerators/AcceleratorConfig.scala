// filename: AcceleratorConfig.scala
package mycnnaccelerators

import chisel3._

// Define data type configurations
sealed trait DataTypeConfig {
  def dataWidth: Int
}
case class FixedPointConfig(dataWidth: Int, fractionBits: Int) extends DataTypeConfig
// Example: case class FloatConfig(dataWidth: Int, expWidth: Int, sigWidth: Int) extends DataTypeConfig

case class AcceleratorConfig(
  // Data types
  ifmDataType: FixedPointConfig = FixedPointConfig(dataWidth = 16, fractionBits = 8),
  kernelDataType: FixedPointConfig = FixedPointConfig(dataWidth = 16, fractionBits = 8),
  ofmDataType: FixedPointConfig = FixedPointConfig(dataWidth = 16, fractionBits = 8),
  accumulatorWidth: Int = 32,

  // Buffer sizes (number of elements)
  ifmDepth: Int = 32 * 32,
  kernelMaxDepth: Int = 5 * 5,
  ofmDepth: Int = 32 * 32,

  // Fixed dimensions
  ifmRows: Int = 32,
  ifmCols: Int = 32,
  kernelMaxRows: Int = 5,
  kernelMaxCols: Int = 5,
  ofmRows: Int = 32,
  ofmCols: Int = 32,

  // DMA parameters
  coreMaxAddrBits: Int = 64,
  tileLinkBeatBytes: Int = 8,

  // Core xLen (processor register width, e.g., 32 or 64)
  // This will be populated by the RoCC wrapper from the core's parameters.
  xLen: Int = 64 // Default value, will be overridden
)

// Default configuration object
// Note: The xLen here is a placeholder if DefaultAcceleratorConfig is used directly
// without being processed by MyCNNRoCC to inject the actual core xLen.
object DefaultAcceleratorConfig extends AcceleratorConfig()
