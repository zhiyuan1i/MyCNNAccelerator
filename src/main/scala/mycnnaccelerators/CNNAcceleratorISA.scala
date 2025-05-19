// filename: CNNAcceleratorISA.scala
package mycnnaccelerators

import chisel3._

object CNNAcceleratorISA {
  // Existing Funct values
  val CMD_SET_IFM_ADDR_DATA    = 0.U(7.W)
  val CMD_SET_KERNEL_ADDR_DATA = 1.U(7.W)
  val CMD_START_COMPUTE        = 2.U(7.W)
  val CMD_GET_OFM_ADDR_DATA    = 3.U(7.W)
  val CMD_GET_STATUS           = 4.U(7.W)

  // New DMA Funct values
  val CMD_DMA_CONFIG_ADDR      = 5.U(7.W) // rs1: main memory base address
  val CMD_DMA_CONFIG_PARAMS    = 6.U(7.W) // rs1: Cat(length_in_bytes (24 bits max suggested), direction (1 bit), buffer_id (2 bits))
  val CMD_DMA_START            = 7.U(7.W) // No operands needed, uses configured values

  // Existing Status codes
  val STATUS_IDLE            = 0.U(8.W)
  val STATUS_LOADING_IFM     = 1.U(8.W) // Will be replaced by DMA status
  val STATUS_LOADING_KERNEL  = 2.U(8.W) // Will be replaced by DMA status
  val STATUS_COMPUTING       = 3.U(8.W)
  val STATUS_COMPUTE_DONE    = 4.U(8.W)
  // val STATUS_ERROR        = 255.U(8.W) // Standard error

  // New DMA Status codes
  val STATUS_DMA_BUSY              = 10.U(8.W) // General DMA busy
  val STATUS_DMA_CONFIG_READY    = 11.U(8.W) // DMA configured, ready to start
  val STATUS_DMA_IFM_LOAD_DONE   = 12.U(8.W)
  val STATUS_DMA_KERNEL_LOAD_DONE= 13.U(8.W)
  val STATUS_DMA_OFM_STORE_DONE  = 14.U(8.W)
  val STATUS_DMA_ERROR           = 254.U(8.W) // DMA specific error
  val STATUS_ERROR               = 255.U(8.W) // General error (keep)


  // Helper objects for DMA configuration
  object BufferIDs {
    val IFM    = 0.U(2.W)
    val KERNEL = 1.U(2.W)
    val OFM    = 2.U(2.W)
    // val MAX_ID = 2.U // For validation if needed
  }

  object DMADirection {
    val MEM_TO_BUF = 0.U(1.W) // Read from Main Memory, Write to Accelerator Buffer
    val BUF_TO_MEM = 1.U(1.W) // Read from Accelerator Buffer, Write to Main Memory
  }

  // Bit widths for packing DMA_CONFIG_PARAMS into rs1 of CMD_DMA_CONFIG_PARAMS
  // Assuming rs1 is xLen, but RoCC commands typically use general purpose registers (e.g., 64-bit)
  // Let's define based on a common register width, e.g., 64.
  // If xLen can be 32, then this packing needs to be conditional or more constrained.
  // For now, assume parameters fit.
  val dmaConfigLenBits    = 24 // Allows up to 16MB transfers
  val dmaConfigDirBits    = 1
  val dmaConfigBufIdBits  = 2
  val dmaConfigTotalParamBits = dmaConfigLenBits + dmaConfigDirBits + dmaConfigBufIdBits
}