// filename: AcceleratorConfig.scala
package mycnnaccelerators

import chisel3.util.log2Ceil

case class AcceleratorConfig(
  dataWidth: Int = 16,
  F_BITS: Int = 8,
  ifmRows: Int = 32,
  ifmCols: Int = 32,
  // kernelRows and kernelCols now represent the MAX supported kernel size for buffer allocation
  kernelRows: Int = 5, // Max kernel rows
  kernelCols: Int = 5, // Max kernel cols
  xLen: Int = 64,
  val isSameConvolution: Boolean = true
) {
  require(F_BITS < dataWidth, "F_BITS must be less than dataWidth")
  require(F_BITS >= 0, "F_BITS cannot be negative")

  val ifmDepth: Int = ifmRows * ifmCols
  // kernelDepth is based on MAX kernel size for buffer dimensioning
  val kernelDepth: Int = kernelRows * kernelCols

  val actualOfmRows: Int = if (isSameConvolution) ifmRows else (ifmRows - kernelRows + 1) // Note: This might need dynamic kernel size if 'valid' conv was used
  val actualOfmCols: Int = if (isSameConvolution) ifmCols else (ifmCols - kernelCols + 1) // Note: This might need dynamic kernel size if 'valid' conv was used
  val ofmDepth: Int = actualOfmRows * actualOfmCols

  require(ifmDepth > 0, "IFM depth must be positive.")
  require(kernelDepth > 0, "Kernel depth must be positive (based on max size).")
  require(ofmDepth > 0, "OFM depth must be positive.")

  val ifmAddrWidth: Int = math.max(1, log2Ceil(ifmDepth))
  // kernelAddrWidth is for the max-sized kernel buffer
  val kernelAddrWidth: Int = math.max(1, log2Ceil(kernelDepth))
  val ofmAddrWidth: Int = math.max(1, log2Ceil(ofmDepth))

  // Helper to calculate kernel depth for a specific dimension
  def getKernelDepthForDim(dim: Int): Int = dim * dim
}

object DefaultAcceleratorConfig extends AcceleratorConfig()