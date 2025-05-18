// filename: CNNAcceleratorISA.scala
package mycnnaccelerators

import chisel3._
import chisel3.util.log2Ceil

object CNNAcceleratorISA {
  // Funct values for RoCC custom instructions (ensure unique within your RoCC opcode space)
  // Assuming a 7-bit funct field as in the original snippet
  val CONFIG_IFM_ADDR           = 0.U(7.W)
  val CONFIG_KERNEL_ADDR_SIZE   = 1.U(7.W) // rs1: addr, rs2: kernel_dim (e.g., 3 for 3x3, 5 for 5x5)
  val CONFIG_OFM_ADDR_PARAMS  = 2.U(7.W) // rs1: addr, rs2: other params if needed (e.g., stride, padding type)
  val START_CONVOLUTION         = 3.U(7.W) // No operands needed if configured prior
  val GET_STATUS                = 4.U(7.W) // Returns status code in rd

  // Status codes returned by GET_STATUS (example values)
  // Ensure these fit within the data width of rd (typically xLen)
  // Using 8 bits for status as in the original
  val STATUS_IDLE               = 0.U(8.W)
  val STATUS_BUSY_DMA_IFM       = 1.U(8.W)
  val STATUS_BUSY_DMA_KERNEL    = 2.U(8.W)
  val STATUS_BUSY_COMPUTE       = 3.U(8.W)
  val STATUS_BUSY_DMA_OFM       = 4.U(8.W)
  val STATUS_DONE_SUCCESS       = 100.U(8.W)
  val STATUS_ERROR_DMA          = 200.U(8.W) // General DMA error
  val STATUS_ERROR_CONFIG       = 201.U(8.W) // Configuration error (e.g., start before full config)
  val STATUS_ERROR_COMPUTE      = 202.U(8.W) // Error during computation (e.g., overflow if detected)
}