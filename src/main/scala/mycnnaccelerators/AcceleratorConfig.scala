// filename: AcceleratorConfig.scala
package mycnnaccelerators

import chisel3.util.log2Ceil

case class AcceleratorConfig(
  dataWidth: Int = 16,
  F_BITS: Int = 8,
  ifmRows: Int = 32,
  ifmCols: Int = 32,
  kernelRows: Int = 5,
  kernelCols: Int = 5,
  xLen: Int = 64,
  // Add a parameter to define convolution type, or assume 'same'
  val isSameConvolution: Boolean = true // Make 'same' the default or set via constructor
) {
  require(F_BITS < dataWidth, "F_BITS must be less than dataWidth")
  require(F_BITS >= 0, "F_BITS cannot be negative")

  val ifmDepth: Int = ifmRows * ifmCols
  val kernelDepth: Int = kernelRows * kernelCols

  // Derive OFM dimensions based on convolution type
  val actualOfmRows: Int = if (isSameConvolution) ifmRows else (ifmRows - kernelRows + 1)
  val actualOfmCols: Int = if (isSameConvolution) ifmCols else (ifmCols - kernelCols + 1)
  val ofmDepth: Int = actualOfmRows * actualOfmCols // Use actualOfmRows/Cols

  require(ifmDepth > 0, "IFM depth must be positive.")
  require(kernelDepth > 0, "Kernel depth must be positive.")
  require(ofmDepth > 0, "OFM depth must be positive.")

  val ifmAddrWidth: Int = math.max(1, log2Ceil(ifmDepth))
  val kernelAddrWidth: Int = math.max(1, log2Ceil(kernelDepth))
  val ofmAddrWidth: Int = math.max(1, log2Ceil(ofmDepth)) // Uses depth from actualOfmRows/Cols

  // Provide access to the OFM dimensions that ComputeUnit will use for its loops and addressing
  // This makes it explicit what dimensions the ComputeUnit should iterate over for its output.
  // In your ComputeUnit, you would then use config.outputRows and config.outputCols for out_r/out_c limits.
  // However, if ComputeUnit directly uses ifmRows/ifmCols for its output loops when doing 'same',
  // then AcceleratorConfig's ofmRows/ofmCols MUST be ifmRows/ifmCols.
  // The key is that the ComputeUnit's loop limits (e.g., config.ifmRows) and the
  // AcceleratorConfig's ofmRows (used by testbench) must match.

  // For clarity, if ComputeUnit uses config.ifmRows for output loops:
  // The testbench should use dutConfig.ifmRows for expected OFM size.
  // The AcceleratorConfig's ofmRows field should reflect config.ifmRows if isSameConvolution is true.
  // The code above with actualOfmRows/actualOfmCols for ofmDepth/ofmAddrWidth is correct.
  // The ComputeUnit should use config.ifmRows/Cols for its out_r/out_c loop limits
  // and for ofm_write_addr calculation like: out_r * config.ifmCols.U + out_c.
  // The testbench will use dutConfig.actualOfmRows/actualOfmCols for its loops and cycle counts.
}

object DefaultAcceleratorConfig extends AcceleratorConfig()