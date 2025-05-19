// filename: AcceleratorConfig.scala
package mycnnaccelerators

import chisel3._ // Not strictly needed in this file if only using Ints and log2Ceil from util
import chisel3.util.log2Ceil

// Basic Accelerator Configuration
case class AcceleratorConfig(
  // Data width for IFM, Kernel, OFM elements (total bits)
  dataWidth: Int = 16,

  // Number of fractional bits for fixed-point data representation
  // Assumes IFM, Kernel, and OFM elements use this format.
  F_BITS: Int = 8, // Example: if dataWidth is 16, this could be Qs7.8 (1 sign, 7 int, 8 frac)

  // IFM Dimensions (fixed)
  ifmRows: Int = 8,
  ifmCols: Int = 8,

  // Kernel Dimensions (fixed)
  kernelRows: Int = 3,
  kernelCols: Int = 3,

  // RoCC Core's xLen (register width, e.g., 64 bits)
  // This will be updated by the RoCC wrapper from the core's parameters.
  xLen: Int = 64 // Default, will be overridden
) {
  // Derived parameters

  // Check for valid F_BITS relative to dataWidth
  // F_BITS must be less than dataWidth (to allow for at least a sign bit or an integer bit)
  require(F_BITS < dataWidth, "F_BITS must be less than dataWidth")
  require(F_BITS >= 0, "F_BITS cannot be negative")

  val ifmDepth: Int = ifmRows * ifmCols
  val kernelDepth: Int = kernelRows * kernelCols

  // OFM Dimensions (calculated for 'valid' convolution, no padding, stride 1)
  val ofmRows: Int = ifmRows - kernelRows + 1
  val ofmCols: Int = ifmCols - kernelCols + 1
  val ofmDepth: Int = ofmRows * ofmCols

  // Address widths for buffers
  // Ensure depth is at least 1 for log2Ceil, or handle zero/negative depths if possible from dimensions
  require(ifmDepth > 0, "IFM depth must be positive to calculate address width.")
  require(kernelDepth > 0, "Kernel depth must be positive to calculate address width.")
  require(ofmDepth > 0, "OFM depth must be positive to calculate address width.")

  val ifmAddrWidth: Int = log2Ceil(ifmDepth)
  val kernelAddrWidth: Int = log2Ceil(kernelDepth)
  val ofmAddrWidth: Int = log2Ceil(ofmDepth)
}

// Default configuration object.
// MyCNNRoCC will typically create the actual config with the correct xLen and potentially other overrides.
object DefaultAcceleratorConfig extends AcceleratorConfig()