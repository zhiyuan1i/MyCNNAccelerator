// filename: AcceleratorConfig.scala
package mycnnaccelerators

import chisel3._
import chisel3.util.log2Ceil

// Basic Accelerator Configuration
case class AcceleratorConfig(
  // Data width for IFM, Kernel, OFM elements
  dataWidth: Int = 8,

  // IFM Dimensions (fixed)
  ifmRows: Int = 8, // Example: Small IFM
  ifmCols: Int = 8,

  // Kernel Dimensions (fixed)
  kernelRows: Int = 3, // Example: 3x3 Kernel
  kernelCols: Int = 3,

  // RoCC Core's xLen (register width, e.g., 64 bits)
  // This will be updated by the RoCC wrapper from the core's parameters.
  xLen: Int = 64 // Default, will be overridden
) {
  // Derived parameters
  val ifmDepth: Int = ifmRows * ifmCols
  val kernelDepth: Int = kernelRows * kernelCols

  // OFM Dimensions (calculated for 'valid' convolution)
  val ofmRows: Int = ifmRows - kernelRows + 1
  val ofmCols: Int = ifmCols - kernelCols + 1
  val ofmDepth: Int = ofmRows * ofmCols

  // Address widths for buffers
  val ifmAddrWidth: Int = log2Ceil(ifmDepth)
  val kernelAddrWidth: Int = log2Ceil(kernelDepth)
  val ofmAddrWidth: Int = log2Ceil(ofmDepth)
}

// Default configuration object, xLen is a placeholder here.
// MyCNNRoCC will create the actual config with the correct xLen.
object DefaultAcceleratorConfig extends AcceleratorConfig()