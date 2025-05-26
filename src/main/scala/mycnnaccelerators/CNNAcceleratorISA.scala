// filename: CNNAcceleratorISA.scala
package mycnnaccelerators

import chisel3._

object CNNAcceleratorISA {
  // Existing Funct values
  val CMD_SET_IFM_ADDR_DATA    = 0.U(7.W) // Still "Not Implemented" in this update
  val CMD_SET_KERNEL_ADDR_DATA = 1.U(7.W) // Still "Not Implemented" in this update
  val CMD_START_COMPUTE        = 2.U(7.W)
  val CMD_GET_OFM_ADDR_DATA    = 3.U(7.W) // Still "Not Implemented" in this update
  val CMD_GET_STATUS           = 4.U(7.W)

  // DMA Funct values
  val CMD_DMA_CONFIG_ADDR      = 5.U(7.W) // rs1: main memory base address
  val CMD_DMA_CONFIG_PARAMS    = 6.U(7.W) // rs1: Cat(length_in_bytes, direction, buffer_id)
  val CMD_DMA_START            = 7.U(7.W) // No operands needed

  // New command to set accelerator parameters, like kernel size
  val CMD_SET_KERNEL_PARAMS    = 8.U(7.W) // rs1: kernel dimension (e.g., 1, 3, 5)

  // Existing Status codes
  val STATUS_IDLE              = 0.U(8.W)
  val STATUS_LOADING_IFM       = 1.U(8.W) // Will be replaced by DMA status
  val STATUS_LOADING_KERNEL    = 2.U(8.W) // Will be replaced by DMA status
  val STATUS_COMPUTING         = 3.U(8.W)
  val STATUS_COMPUTE_DONE      = 4.U(8.W)

  // New DMA Status codes
  val STATUS_DMA_BUSY             = 10.U(8.W) // General DMA busy
  val STATUS_DMA_CONFIG_READY     = 11.U(8.W) // DMA configured, ready to start
  val STATUS_DMA_IFM_LOAD_DONE    = 12.U(8.W)
  val STATUS_DMA_KERNEL_LOAD_DONE = 13.U(8.W)
  val STATUS_DMA_OFM_STORE_DONE   = 14.U(8.W)
  val STATUS_KERNEL_PARAMS_SET    = 15.U(8.W) // New status: Kernel parameters (like size) have been set

  val STATUS_DMA_ERROR         = 254.U(8.W) // DMA specific error
  val STATUS_ERROR             = 255.U(8.W) // General error (keep)


  // Helper objects for DMA configuration
  object BufferIDs {
    val IFM    = 0.U(2.W)
    val KERNEL = 1.U(2.W)
    val OFM    = 2.U(2.W)
  }

  object DMADirection {
    val MEM_TO_BUF = 0.U(1.W) // Read from Main Memory, Write to Accelerator Buffer
    val BUF_TO_MEM = 1.U(1.W) // Read from Accelerator Buffer, Write to Main Memory
  }

  // Bit widths for packing DMA_CONFIG_PARAMS into rs1 of CMD_DMA_CONFIG_PARAMS
  val dmaConfigLenBits      = 24 // Allows up to 16MB transfers
  val dmaConfigDirBits      = 1
  val dmaConfigBufIdBits    = 2
  val dmaConfigTotalParamBits = dmaConfigLenBits + dmaConfigDirBits + dmaConfigBufIdBits

  // For CMD_SET_KERNEL_PARAMS, rs1 could be structured if more params are added.
  // For now, rs1 directly carries the kernel dimension (1, 3, or 5).
  // Example: rs1[2:0] = kernel_dim
}