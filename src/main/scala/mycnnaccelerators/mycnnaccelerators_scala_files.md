## File: MyCNNRoCC.scala

```scala
// filename: MyCNNRoCC.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy.LazyModule
import CNNAcceleratorISA._ // Import ISA definitions

// Key for configuration
case object MyCNNAcceleratorKey extends Field[AcceleratorConfig](DefaultAcceleratorConfig)

class MyCNNRoCC(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  val baseConfig = p(MyCNNAcceleratorKey) // Get base config from Parameters
  // Actual config is created in MyCNNRoCCModuleImp once xLen is known
  override lazy val module = new MyCNNRoCCModuleImp(this, baseConfig)
}

class MyCNNRoCCModuleImp(outer: MyCNNRoCC, defaultConfig: AcceleratorConfig)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters { // HasCoreParameters provides xLen, etc.

  // Create the actual config using xLen from HasCoreParameters
  val config = defaultConfig.copy(xLen = xLen)

  // Instantiate internal modules
  val ifm_buffer = Module(new MinimalBuffer(config.ifmDepth, config.dataWidth))
  val kernel_buffer = Module(new MinimalBuffer(config.kernelDepth, config.dataWidth))
  val ofm_buffer = Module(new MinimalBuffer(config.ofmDepth, config.dataWidth))
  val compute_unit = Module(new ComputeUnit(config))

  // RoCC FSM states
  val sIdle :: sWaitOFMRead :: sRespond :: Nil = Enum(3)
  val rocc_state = RegInit(sIdle)

  // Registers for RoCC interaction
  val resp_data_reg = Reg(UInt(config.xLen.W))
  val resp_rd_reg = Reg(UInt(5.W))
  val resp_valid_reg = RegInit(false.B)

  val accelerator_status_reg = RegInit(STATUS_IDLE) // For GET_STATUS command
  val ofm_read_addr_reg = Reg(UInt(config.ofmAddrWidth.W)) // For CMD_GET_OFM_ADDR_DATA

  // Default outputs for RoCC interface
  io.cmd.ready := (rocc_state === sIdle) && !resp_valid_reg
  io.resp.valid := resp_valid_reg
  io.resp.bits.rd := resp_rd_reg
  io.resp.bits.data := resp_data_reg
  io.busy := (compute_unit.io.busy || accelerator_status_reg === STATUS_COMPUTING) && (rocc_state =/= sIdle)
  io.interrupt := false.B // No interrupts for this simple version

  // Default connections to internal modules
  ifm_buffer.io.write_en := false.B
  ifm_buffer.io.write_addr := 0.U
  ifm_buffer.io.write_data := 0.U
  ifm_buffer.io.read_addr := 0.U // Not directly read by RoCC FSM

  kernel_buffer.io.write_en := false.B
  kernel_buffer.io.write_addr := 0.U
  kernel_buffer.io.write_data := 0.U
  kernel_buffer.io.read_addr := 0.U // Not directly read by RoCC FSM

  ofm_buffer.io.write_en := compute_unit.io.ofm_write_en // CU writes to OFM
  ofm_buffer.io.write_addr := compute_unit.io.ofm_write_addr
  ofm_buffer.io.write_data := compute_unit.io.ofm_write_data
  ofm_buffer.io.read_addr := 0.U // Controlled by RoCC FSM for readback

  compute_unit.io.start := false.B
  compute_unit.io.ifm_read_addr := 0.U(6.W)  // CU controls this
  compute_unit.io.ifm_read_data := ifm_buffer.io.read_data
  compute_unit.io.kernel_read_addr := 0.U // CU controls this
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data

  // RoCC Command Handling FSM
  when(resp_valid_reg && io.resp.ready) {
    resp_valid_reg := false.B // Clear response valid when CPU accepts it
  }

  switch(rocc_state) {
    is(sIdle) {
      accelerator_status_reg := Mux(compute_unit.io.busy, STATUS_COMPUTING,
                                Mux(compute_unit.io.done, STATUS_COMPUTE_DONE, STATUS_IDLE))

      when(io.cmd.valid && !resp_valid_reg) {
        val cmd = io.cmd.bits
        resp_rd_reg := cmd.inst.rd // Capture destination register for all commands

        io.cmd.ready := false.B // Will be set back to true if cmd processed in one cycle

        switch(cmd.inst.funct) {
          is(CMD_SET_IFM_ADDR_DATA) {
            when(cmd.rs1 < config.ifmDepth.U) {
              ifm_buffer.io.write_en   := true.B
              ifm_buffer.io.write_addr := cmd.rs1
              ifm_buffer.io.write_data := cmd.rs2(config.dataWidth - 1, 0) // Assuming data fits in rs2
              accelerator_status_reg := STATUS_LOADING_IFM // Optional
              // This is a single cycle write to buffer command, no CPU response needed or status can be polled
            } .otherwise {
              accelerator_status_reg := STATUS_ERROR // Address out of bounds
            }
            io.cmd.ready := true.B // Ready for next command immediately
          }
          is(CMD_SET_KERNEL_ADDR_DATA) {
            when(cmd.rs1 < config.kernelDepth.U) {
              kernel_buffer.io.write_en   := true.B
              kernel_buffer.io.write_addr := cmd.rs1
              kernel_buffer.io.write_data := cmd.rs2(config.dataWidth - 1, 0)
              accelerator_status_reg := STATUS_LOADING_KERNEL // Optional
            } .otherwise {
              accelerator_status_reg := STATUS_ERROR
            }
            io.cmd.ready := true.B
          }
          is(CMD_START_COMPUTE) {
            when(!compute_unit.io.busy) {
              compute_unit.io.start := true.B
              accelerator_status_reg := STATUS_COMPUTING
            }
            // No direct response, status polled via GET_STATUS
            io.cmd.ready := true.B
          }
          is(CMD_GET_OFM_ADDR_DATA) {
            when(cmd.rs1 < config.ofmDepth.U) {
              ofm_buffer.io.read_addr := cmd.rs1
              ofm_read_addr_reg := cmd.rs1 // Save for data fetch next cycle
              rocc_state := sWaitOFMRead
            } .otherwise {
              resp_data_reg := STATUS_ERROR.asUInt // Or specific error code
              resp_valid_reg := true.B
              accelerator_status_reg := STATUS_ERROR
              rocc_state := sIdle // or sRespond to send error
            }
          }
          is(CMD_GET_STATUS) {
            // Update status based on CU state before responding
            val current_status = Mux(compute_unit.io.busy, STATUS_COMPUTING,
                                 Mux(compute_unit.io.done && accelerator_status_reg =/= STATUS_IDLE, STATUS_COMPUTE_DONE, // Latch compute_done if it was busy
                                     accelerator_status_reg)) // else keep old status (like error or idle)
            resp_data_reg := current_status
            resp_valid_reg := true.B
            rocc_state := sIdle // or sRespond
            // If compute_unit.io.done is high, and we were computing, update main status
            when(compute_unit.io.done && accelerator_status_reg === STATUS_COMPUTING) {
                accelerator_status_reg := STATUS_COMPUTE_DONE
            }
          }
        }
      }
    }

    is(sWaitOFMRead) {
      // Data from ofm_buffer.io.read_data is valid in this cycle
      // (since address was set in sIdle the previous cycle)
      resp_data_reg := ofm_buffer.io.read_data
      resp_valid_reg := true.B
      rocc_state := sRespond // Go to respond state to wait for CPU ack
    }

    is(sRespond) {
      // Wait for resp_valid_reg to be cleared by CPU io.resp.ready
      when(!resp_valid_reg) {
        io.cmd.ready := true.B
        rocc_state := sIdle
      }
    }
  }

  // Connect Compute Unit to Buffers (CU drives addresses, SPADs provide data after 1 cycle)
  compute_unit.io.ifm_read_data    := ifm_buffer.io.read_data
  compute_unit.io.kernel_read_data := kernel_buffer.io.read_data

  // CU drives its own read addresses to the buffers
  ifm_buffer.io.read_addr    := compute_unit.io.ifm_read_addr
  kernel_buffer.io.read_addr := compute_unit.io.kernel_read_addr

  // When compute unit is done, and RoCC was in computing state, update RoCC's status
   when(compute_unit.io.done && accelerator_status_reg === STATUS_COMPUTING) {
       accelerator_status_reg := STATUS_COMPUTE_DONE
   }
   // If RoCC gets a start command when CU is already done, reset CU's done for next run
   when(io.cmd.fire && io.cmd.bits.inst.funct === CMD_START_COMPUTE && compute_unit.io.done) {
       // This is handled by CU's sDone->sIdle transition on !io.start
   }
}
```

## File: MinimalBuffer.scala

```scala
// filename: MinimalBuffer.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class MinimalBufferIO(addrWidth: Int, dataWidth: Int) extends Bundle {
  // Write port
  val write_en   = Input(Bool())
  val write_addr = Input(UInt(addrWidth.W))
  val write_data = Input(UInt(dataWidth.W))

  // Read port
  // For SyncReadMem, read data is available one cycle after 'read_addr' is asserted
  // if an explicit read_en for the mem itself is true, or if read is always enabled based on addr.
  val read_addr  = Input(UInt(addrWidth.W))
  val read_data  = Output(UInt(dataWidth.W)) // Data will be valid next cycle
}

class MinimalBuffer(depth: Int, dataWidth: Int) extends Module {
  val addrWidth = log2Ceil(depth)
  val io = IO(new MinimalBufferIO(addrWidth, dataWidth))

  val mem = SyncReadMem(depth, UInt(dataWidth.W))

  when(io.write_en) {
    mem.write(io.write_addr, io.write_data)
  }

  // The read operation from SyncReadMem is registered.
  // Data appears on io.read_data in the cycle *after* io.read_addr is set.
  // The read enable for SyncReadMem.read is implicitly true here.
  io.read_data := mem.read(io.read_addr, true.B)
}
```

## File: AcceleratorConfig.scala

```scala
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
```

## File: ComputeUnit.scala

```scala
// filename: ComputeUnit.scala
package mycnnaccelerators

import chisel3._
import chisel3.util._

class ComputeUnitIO(val config: AcceleratorConfig) extends Bundle {
  val start = Input(Bool())
  val busy  = Output(Bool())
  val done  = Output(Bool())

  // IFM Buffer Interface (CU reads from IFM buffer)
  val ifm_read_addr = Output(UInt(config.ifmAddrWidth.W))
  val ifm_read_data = Input(UInt(config.dataWidth.W)) // Data from IFM buffer

  // Kernel Buffer Interface (CU reads from Kernel buffer)
  val kernel_read_addr = Output(UInt(config.kernelAddrWidth.W))
  val kernel_read_data = Input(UInt(config.dataWidth.W)) // Data from Kernel buffer

  // OFM Buffer Interface (CU writes to OFM buffer)
  val ofm_write_en   = Output(Bool())
  val ofm_write_addr = Output(UInt(config.ofmAddrWidth.W))
  val ofm_write_data = Output(UInt(config.dataWidth.W))
}

class ComputeUnit(val config: AcceleratorConfig) extends Module {
  val io = IO(new ComputeUnitIO(config))

  // States for the Compute Unit FSM
  val sIdle :: sFetchIFM :: sFetchKernel :: sMAC :: sWriteOFM :: sDoneCU :: Nil = Enum(6)
  val state = RegInit(sIdle)

  // Registers for loop counters and data
  val ofm_row = RegInit(0.U(log2Ceil(config.ofmRows).W))
  val ofm_col = RegInit(0.U(log2Ceil(config.ofmCols).W))

  val k_row = RegInit(0.U(log2Ceil(config.kernelRows).W))
  val k_col = RegInit(0.U(log2Ceil(config.kernelCols).W))

  // Accumulator - make it wider to prevent overflow during MAC
  val accWidth = config.dataWidth * 2 + log2Ceil(config.kernelRows * config.kernelCols)
  val accumulator = RegInit(0.S(accWidth.W)) // Signed for potential future use, works for UInt too

  // Registers to hold data read from buffers (due to SyncReadMem 1-cycle latency)
  val ifm_data_reg   = Reg(UInt(config.dataWidth.W))
  val kernel_data_reg = Reg(UInt(config.dataWidth.W))
  val data_fetch_cycle = RegInit(false.B) // To manage 2-cycle read

  // Default outputs
  io.busy := (state =/= sIdle) && (state =/= sDoneCU)
  io.done := (state === sDoneCU)

  io.ifm_read_addr := 0.U
  io.kernel_read_addr := 0.U

  io.ofm_write_en   := false.B
  io.ofm_write_addr := 0.U
  io.ofm_write_data := accumulator(config.dataWidth - 1, 0).asUInt // Truncate/saturate as needed

  switch(state) {
    is(sIdle) {
      when(io.start) {
        ofm_row := 0.U
        ofm_col := 0.U
        k_row   := 0.U
        k_col   := 0.U
        accumulator := 0.S
        state := sFetchIFM
        data_fetch_cycle := false.B // Start fetch cycle 0 (addressing)
      }
    }

    is(sFetchIFM) {
      // Cycle 0: Assert address for IFM data
      // Cycle 1: Latch data, assert address for Kernel data
      when(!data_fetch_cycle) { // Cycle 0
        val current_ifm_row = ofm_row + k_row
        val current_ifm_col = ofm_col + k_col
        io.ifm_read_addr := current_ifm_row * config.ifmCols.U + current_ifm_col
        data_fetch_cycle := true.B
        // Stay in sFetchIFM for next cycle to latch data
      } .otherwise { // Cycle 1
        ifm_data_reg := io.ifm_read_data // Latch IFM data from previous cycle's address
        state := sFetchKernel
        data_fetch_cycle := false.B // Reset for kernel fetch
      }
    }

    is(sFetchKernel) {
      // Cycle 0: Assert address for Kernel data
      // Cycle 1: Latch data, proceed to MAC
      when(!data_fetch_cycle) { // Cycle 0
        io.kernel_read_addr := k_row * config.kernelCols.U + k_col
        data_fetch_cycle := true.B
      } .otherwise { // Cycle 1
        kernel_data_reg := io.kernel_read_data // Latch Kernel data
        state := sMAC
        data_fetch_cycle := false.B
      }
    }

    is(sMAC) {
      accumulator := accumulator + (ifm_data_reg.zext * kernel_data_reg.zext).asSInt

      // Increment kernel iterators
      k_col := k_col + 1.U
      when(k_col + 1.U === config.kernelCols.U) {
        k_col := 0.U
        k_row := k_row + 1.U
        when(k_row + 1.U === config.kernelRows.U) {
          k_row := 0.U
          state := sWriteOFM // All kernel elements processed for this OFM pixel
        } .otherwise {
          state := sFetchIFM // Next row in kernel
        }
      } .otherwise {
        state := sFetchIFM // Next col in kernel
      }
    }

    is(sWriteOFM) {
      io.ofm_write_en   := true.B
      io.ofm_write_addr := ofm_row * config.ofmCols.U + ofm_col
      // Data is set by default connection. Ensure accumulator is appropriately sized and truncated.

      accumulator := 0.S // Reset accumulator for next OFM element

      // Increment OFM iterators
      ofm_col := ofm_col + 1.U
      when(ofm_col + 1.U === config.ofmCols.U) {
        ofm_col := 0.U
        ofm_row := ofm_row + 1.U
        when(ofm_row + 1.U === config.ofmRows.U) {
          state := sDoneCU // All OFM pixels computed
        } .otherwise {
          k_row := 0.U // Reset kernel iterators for new OFM element
          k_col := 0.U
          state := sFetchIFM // Next row in OFM
        }
      } .otherwise {
        k_row := 0.U // Reset kernel iterators for new OFM element
        k_col := 0.U
        state := sFetchIFM // Next col in OFM
      }
    }

    is(sDoneCU) {
      when(!io.start) { // Optional: wait for start to de-assert before idling
        state := sIdle
      }
    }
  }
}
```

## File: CNNAcceleratorISA.scala

```scala
// filename: CNNAcceleratorISA.scala
package mycnnaccelerators

import chisel3._

object CNNAcceleratorISA {
  // Funct values for RoCC custom instructions (7-bit)
  // rs1 generally for address, rs2 for data (if applicable)
  // rd generally for response data (if applicable)

  val CMD_SET_IFM_ADDR_DATA   = 0.U(7.W) // rs1: address in IFM buffer, rs2: data
  val CMD_SET_KERNEL_ADDR_DATA= 1.U(7.W) // rs1: address in Kernel buffer, rs2: data
  val CMD_START_COMPUTE       = 2.U(7.W) // No operands needed if data is pre-loaded
  val CMD_GET_OFM_ADDR_DATA   = 3.U(7.W) // rs1: address in OFM buffer, rd: data from OFM
  val CMD_GET_STATUS          = 4.U(7.W) // rd: status code

  // Status codes (ensure fits in xLen, typically 8-bits are plenty)
  val STATUS_IDLE             = 0.U(8.W)
  val STATUS_LOADING_IFM      = 1.U(8.W) // Optional: if we want finer grain status
  val STATUS_LOADING_KERNEL   = 2.U(8.W) // Optional
  val STATUS_COMPUTING        = 3.U(8.W)
  val STATUS_COMPUTE_DONE     = 4.U(8.W) // Computation finished, OFM is ready
  val STATUS_ERROR            = 255.U(8.W) // General error
}
```

