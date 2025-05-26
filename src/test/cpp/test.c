// filename: my_cnn_rocc_test.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h> // For malloc/free if used, though static allocation is simpler for this test
#include "generated_conv_data.h"

// RoCC Instruction Header (from your demo)
// Based on code by Schuyler Eldridge. Copyright (c) Boston University
// https://github.com/seldridge/rocket-rocc-examples/blob/master/src/main/c/rocc.h
#ifndef SRC_MAIN_C_ROCC_H // Include guard for rocc.h content
#define SRC_MAIN_C_ROCC_H

#include <riscv-pk/encoding.h> // May not be available directly, rdcycle is usually via CSR
                               // For baremetal/sim, we might need a different way to get cycle counts
                               // or rely on simulator features. For now, keep as is.

#define STR1(x) #x
#define STR(x) STR1(x)
#define EXTRACT(a, size, offset) (((~(~0 << size) << offset) & a) >> offset)

#define CUSTOMX_OPCODE(x) CUSTOM_ ## x
#define CUSTOM_0 0b0001011
#define CUSTOM_1 0b0101011
#define CUSTOM_2 0b1011011
#define CUSTOM_3 0b1111011

// CUSTOMX macro defines the raw instruction encoding
#define CUSTOMX(X, xd, xs1, xs2, rd, rs1, rs2, funct) \
  CUSTOMX_OPCODE(X)                           | \
  (rd                           << (7))        | \
  (EXTRACT(funct, 7, 0) << (7+5+3+5+5))   | /* funct7 */ \
  (rs2                          << (7+5+3+5))    | /* rs2 */ \
  (rs1                          << (7+5+3))      | /* rs1 */ \
  (0b000                        << (7+5))        | /* funct3 (hardcoded to 000 for RoCC) */ \
  (xd                           << (7+5+1+1))    | /* xd (destination register used) */ \
  (xs1                          << (7+5+1))      | /* xs1 (source register 1 used) */ \
  (xs2                          << (7+5+0))        /* xs2 (source register 2 used) */


// Standard macro that passes rd, rs1, and rs2 via registers
// It uses general purpose registers x10, x11, x12 for rd, rs1, rs2 respectively.
// Adjust register numbers if needed, or if specific registers are required by ABI.
#define ROCC_INSTRUCTION_DSS(X, rd_val, rs1_val, rs2_val, funct) \
    ROCC_INSTRUCTION_R_R_R(X, rd_val, rs1_val, rs2_val, funct, 10, 11, 12)

#define ROCC_INSTRUCTION_DS(X, rd_val, rs1_val, funct) \
    ROCC_INSTRUCTION_R_R_I(X, rd_val, rs1_val, 0, funct, 10, 11)

#define ROCC_INSTRUCTION_D(X, rd_val, funct) \
    ROCC_INSTRUCTION_R_I_I(X, rd_val, 0, 0, funct, 10)

#define ROCC_INSTRUCTION_SS(X, rs1_val, rs2_val, funct) \
    ROCC_INSTRUCTION_I_R_R(X, 0, rs1_val, rs2_val, funct, 11, 12)

#define ROCC_INSTRUCTION_S(X, rs1_val, funct) \
    ROCC_INSTRUCTION_I_R_I(X, 0, rs1_val, 0, funct, 11)

#define ROCC_INSTRUCTION(X, funct) \
    ROCC_INSTRUCTION_I_I_I(X, 0, 0, 0, funct)


// rd, rs1, and rs2 are data values
// rd_n, rs1_n, and rs2_n are the register numbers to use (e.g., 10 for x10)
#define ROCC_INSTRUCTION_R_R_R(X, rd_val, rs1_val, rs2_val, funct, rd_n, rs1_n, rs2_n) { \
    register uint64_t rd_ asm ("x" # rd_n); \
    register uint64_t rs1_ asm ("x" # rs1_n) = (uint64_t) rs1_val; \
    register uint64_t rs2_ asm ("x" # rs2_n) = (uint64_t) rs2_val; \
    asm volatile ( \
        ".word " STR(CUSTOMX(X, 1, 1, 1, rd_n, rs1_n, rs2_n, funct)) "\n\t" \
        : "=r" (rd_) \
        : [_rs1] "r" (rs1_), [_rs2] "r" (rs2_)); \
    rd_val = rd_; \
}

// rd, rs1 are data values; rs2 is an immediate (not used by RoCC command encoding directly, but part of macro)
#define ROCC_INSTRUCTION_R_R_I(X, rd_val, rs1_val, rs2_imm, funct, rd_n, rs1_n) { \
    register uint64_t rd_ asm ("x" # rd_n); \
    register uint64_t rs1_ asm ("x" # rs1_n) = (uint64_t) rs1_val; \
    asm volatile ( \
        ".word " STR(CUSTOMX(X, 1, 1, 0, rd_n, rs1_n, rs2_imm, funct)) "\n\t" \
        : "=r" (rd_) \
        : [_rs1] "r" (rs1_)); \
    rd_val = rd_; \
}

// rd is data value; rs1 and rs2 are immediates
#define ROCC_INSTRUCTION_R_I_I(X, rd_val, rs1_imm, rs2_imm, funct, rd_n) { \
    register uint64_t rd_ asm ("x" # rd_n); \
    asm volatile ( \
        ".word " STR(CUSTOMX(X, 1, 0, 0, rd_n, rs1_imm, rs2_imm, funct)) "\n\t" \
        : "=r" (rd_)); \
    rd_val = rd_; \
}

// rs1, rs2 are data values; rd is an immediate (register number)
#define ROCC_INSTRUCTION_I_R_R(X, rd_imm, rs1_val, rs2_val, funct, rs1_n, rs2_n) { \
    register uint64_t rs1_ asm ("x" # rs1_n) = (uint64_t) rs1_val; \
    register uint64_t rs2_ asm ("x" # rs2_n) = (uint64_t) rs2_val; \
    asm volatile ( \
        ".word " STR(CUSTOMX(X, 0, 1, 1, rd_imm, rs1_n, rs2_n, funct)) "\n\t" \
        :: [_rs1] "r" (rs1_), [_rs2] "r" (rs2_)); \
}

// rs1 is data value; rd, rs2 are immediates
#define ROCC_INSTRUCTION_I_R_I(X, rd_imm, rs1_val, rs2_imm, funct, rs1_n) { \
    register uint64_t rs1_ asm ("x" # rs1_n) = (uint64_t) rs1_val; \
    asm volatile ( \
        ".word " STR(CUSTOMX(X, 0, 1, 0, rd_imm, rs1_n, rs2_imm, funct)) "\n\t" \
        :: [_rs1] "r" (rs1_)); \
}

// rd, rs1, rs2 are immediates
#define ROCC_INSTRUCTION_I_I_I(X, rd_imm, rs1_imm, rs2_imm, funct) { \
    asm volatile ( \
        ".word " STR(CUSTOMX(X, 0, 0, 0, rd_imm, rs1_imm, rs2_imm, funct)) "\n\t" ); \
}

#endif // SRC_MAIN_C_ROCC_H


// CNN Accelerator ISA definitions (mirrored from CNNAcceleratorISA.scala)
// Funct values
#define CMD_SET_IFM_ADDR_DATA     0
#define CMD_SET_KERNEL_ADDR_DATA  1
#define CMD_START_COMPUTE         2
#define CMD_GET_OFM_ADDR_DATA     3
#define CMD_GET_STATUS            4
#define CMD_DMA_CONFIG_ADDR       5
#define CMD_DMA_CONFIG_PARAMS     6
#define CMD_DMA_START             7

// Status codes
#define STATUS_IDLE                 0
#define STATUS_COMPUTING            3
#define STATUS_COMPUTE_DONE         4
#define STATUS_DMA_BUSY             10
#define STATUS_DMA_CONFIG_READY     11
#define STATUS_DMA_IFM_LOAD_DONE    12
#define STATUS_DMA_KERNEL_LOAD_DONE 13
#define STATUS_DMA_OFM_STORE_DONE   14
#define STATUS_DMA_ERROR            254
#define STATUS_ERROR                255

// Buffer IDs
#define BUFFER_ID_IFM    0
#define BUFFER_ID_KERNEL 1
#define BUFFER_ID_OFM    2

// DMA Directions
#define DMA_DIR_MEM_TO_BUF 0
#define DMA_DIR_BUF_TO_MEM 1

// DMA Param packing bit positions (from CNNAcceleratorISA.scala)
#define DMA_PARAM_BUF_ID_BITS    2
#define DMA_PARAM_DIR_BITS       1
// Total bits for buffer_id and direction
#define DMA_PARAM_BUF_ID_DIR_TOTAL_BITS (DMA_PARAM_BUF_ID_BITS + DMA_PARAM_DIR_BITS)


// Helper function to read cycle counter
// This is a standard way to read cycles in RISC-V.
static inline uint64_t rdcycle_custom(void) {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// Wrapper functions for RoCC commands
// We use CUSTOM_0 as the RoCC opcode. This might need to change based on your SoC configuration.
static inline void rocc_dma_config_addr(uint64_t addr) {
    uint64_t dummy_rd = 0; // We don't expect a useful return value in rd for this command
    ROCC_INSTRUCTION_DSS(0, dummy_rd, addr, 0, CMD_DMA_CONFIG_ADDR);
}

static inline void rocc_dma_config_params(uint64_t len_bytes, uint64_t direction, uint64_t buffer_id) {
    uint64_t params = (len_bytes << DMA_PARAM_BUF_ID_DIR_TOTAL_BITS) |
                      (direction << DMA_PARAM_BUF_ID_BITS) |
                      buffer_id;
    uint64_t dummy_rd = 0;
    ROCC_INSTRUCTION_DSS(0, dummy_rd, params, 0, CMD_DMA_CONFIG_PARAMS);
}

static inline void rocc_dma_start() {
    uint64_t dummy_rd = 0;
    ROCC_INSTRUCTION_DSS(0, dummy_rd, 0, 0, CMD_DMA_START);
}

static inline void rocc_start_compute() {
    uint64_t dummy_rd = 0;
    ROCC_INSTRUCTION_DSS(0, dummy_rd, 0, 0, CMD_START_COMPUTE);
}

static inline uint64_t rocc_get_status() {
    uint64_t status;
    ROCC_INSTRUCTION_DSS(0, status, 0, 0, CMD_GET_STATUS);
    return status;
}

// Define matrix and data parameters
#define IFM_ROWS 32
#define IFM_COLS 32
#define KERNEL_ROWS 5
#define KERNEL_COLS 5
#define OFM_ROWS IFM_ROWS
#define OFM_COLS IFM_COLS
#define DATA_WIDTH_BYTES 2 // 16-bit data

// Static allocation for simplicity in simulation
// Use pre-generated data (pointers to const data)
const int16_t *ifm_data = generated_ifm_data;
const int16_t *kernel_data = generated_kernel_data;
const int16_t *ofm_expected_data = generated_ofm_expected_data;

// Accelerator still writes to ofm_data, so it needs to be writable
// and correctly sized based on your C defines (which should match generated defines)
int16_t ofm_data[IFM_ROWS * IFM_COLS];

long int manual_round(double val) {
    double result;
    if (val >= 0.0) {
        result = val + 0.5;
    } else {
        result = val - 0.5;
    }
    return (long int)result; // Truncation achieves rounding after adding/subtracting 0.5
}

int16_t float_to_sint16_fixed(double val, int f_bits) {
    double scaled_val = val * (1 << f_bits);
    long int rounded_scaled_val = manual_round(scaled_val); // Use the manual version

    // Saturation for int16_t
    if (rounded_scaled_val > 32767) {
        return 32767;
    }
    if (rounded_scaled_val < -32768) {
        return -32768;
    }
    return (int16_t)rounded_scaled_val;
}

// ***** MODIFIED FUNCTION TO USE FIXED DATA *****
// void initialize_data_fixed_point(const int f_bits) {
//     printf("Initializing IFM and Kernel data with FIXED values (F_BITS=%d)...\n", f_bits);

//     // Initialize IFM with a fixed value (e.g., 1.0, which is 256 for F_BITS=8)
//     // Or -5.0 which is -1280 for F_BITS=8 to match your earlier expected IFM[0] for debugging.
//     // Let's use -1280 for ifm_data[0] and a different fixed value for others to see patterns.
//     // For simplicity in this example, let's make most IFM elements 1.0 (256)
//     // and a few specific ones like your original ifm_data[0].
//     int16_t fixed_ifm_val = float_to_sint16_fixed(1.0, f_bits); // e.g., 256 if f_bits=8
//     for (int i = 0; i < IFM_ROWS * IFM_COLS; ++i) {
//         ifm_data[i] = fixed_ifm_val;
//     }
//     // Override a few specific values if needed for targeted debugging
//     // For example, to match your expected -1280 for ifm_data[0]:
//     ifm_data[0] = float_to_sint16_fixed(-5.0, f_bits); // This will be -1280 if f_bits=8
//     if (IFM_ROWS * IFM_COLS > 1) {
//       ifm_data[1] = float_to_sint16_fixed(2.5, f_bits); // Example: 640 if f_bits=8
//     }


//     // Initialize Kernel with a fixed value (e.g., 0.5, which is 128 for F_BITS=8)
//     // Or -1.0 which is -256 for F_BITS=8 to match your earlier expected KERNEL[0].
//     int16_t fixed_kernel_val = float_to_sint16_fixed(0.5, f_bits); // e.g., 128 if f_bits=8
//     for (int i = 0; i < KERNEL_ROWS * KERNEL_COLS; ++i) {
//         kernel_data[i] = fixed_kernel_val;
//     }
//     // Override a few specific values if needed
//     kernel_data[0] = float_to_sint16_fixed(-1.0, f_bits); // This will be -256 if f_bits=8
//     if (KERNEL_ROWS * KERNEL_COLS > 1) {
//         kernel_data[1] = float_to_sint16_fixed(0.25, f_bits); // Example: 64 if f_bits=8
//     }


//     for (int i = 0; i < OFM_ROWS * OFM_COLS; ++i) {
//         ofm_data[i] = 0; // Clear OFM buffer
//         ofm_expected_data[i] = 0; // Clear expected OFM buffer
//     }
//     printf("Fixed data initialization complete.\n");
//     printf("Example: ifm_data[0] = %d (should be -1280 if F_BITS=8)\n", ifm_data[0]);
//     printf("Example: kernel_data[0] = %d (should be -256 if F_BITS=8)\n", kernel_data[0]);
// }
// ***** END OF MODIFIED FUNCTION *****

// // Simple software 'same' convolution for verification
// void software_convolution_reference_fixed_point(const int f_bits) {
//     printf("Calculating software reference convolution (fixed-point aware, F_BITS=%d)...\n", f_bits);
//     int pad_rows = (KERNEL_ROWS - 1) / 2;
//     int pad_cols = (KERNEL_COLS - 1) / 2;

//     for (int orow = 0; orow < OFM_ROWS; ++orow) {
//         for (int ocol = 0; ocol < OFM_COLS; ++ocol) {
//             int64_t accumulator_2f_bits = 0;

//             for (int krow = 0; krow < KERNEL_ROWS; ++krow) {
//                 for (int kcol = 0; kcol < KERNEL_COLS; ++kcol) {
//                     int ifm_row_eff = orow - pad_rows + krow;
//                     int ifm_col_eff = ocol - pad_cols + kcol;

//                     if (ifm_row_eff >= 0 && ifm_row_eff < IFM_ROWS &&
//                         ifm_col_eff >= 0 && ifm_col_eff < IFM_COLS) {
//                         int32_t ifm_val_scaled = ifm_data[ifm_row_eff * IFM_COLS + ifm_col_eff];
//                         int32_t krn_val_scaled = kernel_data[krow * KERNEL_COLS + kcol];
//                         accumulator_2f_bits += (int64_t)ifm_val_scaled * krn_val_scaled;
//                     }
//                 }
//             }
//             int32_t ofm_val_scaled_f_bits = (int32_t)(accumulator_2f_bits >> f_bits);

//             if (ofm_val_scaled_f_bits > 32767) {
//                 ofm_expected_data[orow * OFM_COLS + ocol] = 32767;
//             } else if (ofm_val_scaled_f_bits < -32768) {
//                 ofm_expected_data[orow * OFM_COLS + ocol] = -32768;
//             } else {
//                 ofm_expected_data[orow * OFM_COLS + ocol] = (int16_t)ofm_val_scaled_f_bits;
//             }
//         }
//     }
//     printf("Software reference (fixed-point aware) calculation complete.\n");
// }


// Function to poll status until a desired status is reached or timeout
int poll_status_until(uint64_t target_status, uint64_t error_status1, uint64_t error_status2, const char* operation_name) {
    uint64_t current_status;
    int attempts = 0;
    const int max_attempts = 1000000; // Timeout to prevent infinite loop

    printf("Polling for %s completion (target status: %lu)...\n", operation_name, target_status);
    do {
        current_status = rocc_get_status();
        if (current_status == target_status) {
            printf("%s completed successfully. Status: %lu\n", operation_name, current_status);
            return 0; // Success
        }
        if (current_status == error_status1 || current_status == error_status2) {
            printf("Error during %s. Status: %lu\n", operation_name, current_status);
            return -1; // Error
        }
        attempts++;
        if (attempts > 0 && (attempts % (max_attempts/20) == 0 || attempts == 1) ) { // Print more frequently
             printf("Still waiting for %s... Current Status: %lu, Target: %lu, Attempts: %d/%d\n",
                    operation_name, current_status, target_status, attempts, max_attempts);
        }

    } while (attempts < max_attempts);

    printf("Timeout waiting for %s. Last status: %lu\n", operation_name, current_status);
    return -1; // Timeout
}


int main(void) {
    uint64_t start_cycle, end_cycle;
    uint64_t current_status;

    printf("Starting CNN Accelerator RoCC Test\n");

    // initialize_data_fixed_point(8); // Using F_BITS = 8 as in your previous logs
    // --- Physical addresses (placeholders - these would come from linker/memory map in a real system) ---
    // In simulation, these are just identifiers. The simulator maps them.
    // Ensure these are aligned if your DMA/memory system requires it.
    uint64_t ifm_addr    = (uint64_t)ifm_data;
    uint64_t kernel_addr = (uint64_t)kernel_data;
    uint64_t ofm_addr    = (uint64_t)ofm_data;

    printf("IFM data memory address: 0x%lx\n", ifm_addr);
    printf("Kernel data memory address: 0x%lx\n", kernel_addr);
    printf("OFM data memory address: 0x%lx\n", ofm_addr);

    uint64_t ifm_len_bytes    = IFM_ROWS * IFM_COLS * DATA_WIDTH_BYTES;
    uint64_t kernel_len_bytes = KERNEL_ROWS * KERNEL_COLS * DATA_WIDTH_BYTES;
    uint64_t ofm_len_bytes    = OFM_ROWS * OFM_COLS * DATA_WIDTH_BYTES;

    printf("Calculated IFM length: %lu bytes\n", ifm_len_bytes);
    printf("Calculated Kernel length: %lu bytes\n", kernel_len_bytes);
    printf("Calculated OFM length: %lu bytes\n", ofm_len_bytes);

    start_cycle = rdcycle_custom();
    printf("Start cycle count: %lu\n", start_cycle);

    // 1. Load IFM
    printf("--- Step 1: Configure and Load IFM ---\n");
    printf("Sending CMD_DMA_CONFIG_ADDR for IFM with address: 0x%lx\n", ifm_addr);
    rocc_dma_config_addr(ifm_addr);
    current_status = rocc_get_status(); // Get status immediately after config_addr
    // printf("Status after CMD_DMA_CONFIG_ADDR for IFM: %lu (Expected: %d for Idle or %d for ConfigReady if it sets it)\n", current_status, STATUS_IDLE, STATUS_DMA_CONFIG_READY);

    printf("Sending CMD_DMA_CONFIG_PARAMS for IFM. Length: %lu, Direction: MEM_TO_BUF (%d), BufferID: IFM (%d)\n", ifm_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_IFM);
    rocc_dma_config_params(ifm_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_IFM);
    current_status = rocc_get_status();
    printf("Status after CMD_DMA_CONFIG_PARAMS for IFM: %lu\n", current_status);
    if (current_status != STATUS_DMA_CONFIG_READY) {
        printf("Error: Expected STATUS_DMA_CONFIG_READY (%d) after IFM params, got %lu\n", STATUS_DMA_CONFIG_READY, current_status);
        return 1;
    }
    printf("IFM DMA configuration successful. Sending CMD_DMA_START for IFM.\n");
    rocc_dma_start();
    current_status = rocc_get_status(); // Status right after start command
    // printf("Status immediately after CMD_DMA_START for IFM: %lu (Expected: %d for DMA_BUSY)\n", current_status, STATUS_DMA_BUSY);
    if (poll_status_until(STATUS_DMA_IFM_LOAD_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "IFM Load") != 0) return 1;
    printf("--- IFM Load Complete ---\n");

    // 2. Load Kernel
    printf("--- Step 2: Configure and Load Kernel ---\n");
    printf("Sending CMD_DMA_CONFIG_ADDR for Kernel with address: 0x%lx\n", kernel_addr);
    rocc_dma_config_addr(kernel_addr);
    current_status = rocc_get_status();
    printf("Status after CMD_DMA_CONFIG_ADDR for Kernel: %lu\n", current_status);

    printf("Sending CMD_DMA_CONFIG_PARAMS for Kernel. Length: %lu, Direction: MEM_TO_BUF (%d), BufferID: KERNEL (%d)\n", kernel_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_KERNEL);
    rocc_dma_config_params(kernel_len_bytes, DMA_DIR_MEM_TO_BUF, BUFFER_ID_KERNEL);
    current_status = rocc_get_status();
    printf("Status after CMD_DMA_CONFIG_PARAMS for Kernel: %lu\n", current_status);
      if (current_status != STATUS_DMA_CONFIG_READY) {
        printf("Error: Expected STATUS_DMA_CONFIG_READY (%d) after Kernel params, got %lu\n", STATUS_DMA_CONFIG_READY, current_status);
        return 1;
    }
    printf("Kernel DMA configuration successful. Sending CMD_DMA_START for Kernel.\n");
    rocc_dma_start();
    current_status = rocc_get_status();
    // printf("Status immediately after CMD_DMA_START for Kernel: %lu (Expected: %d for DMA_BUSY)\n", current_status, STATUS_DMA_BUSY);
    if (poll_status_until(STATUS_DMA_KERNEL_LOAD_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "Kernel Load") != 0) return 1;
    printf("--- Kernel Load Complete ---\n");

    // 3. Start Computation
    printf("--- Step 3: Start Computation ---\n");
    printf("Sending CMD_START_COMPUTE.\n");
    rocc_start_compute();
    printf("Waiting for computation to start...\n");
    current_status = rocc_get_status(); // Status right after start compute command
    // printf("Status immediately after CMD_START_COMPUTE: %lu (Expected: %d for COMPUTING or an intermediate state)\n", current_status, STATUS_COMPUTING);
    if (poll_status_until(STATUS_COMPUTE_DONE, STATUS_ERROR, STATUS_DMA_ERROR, "Computation") != 0) return 1;
    printf("--- Computation Complete ---\n");

    // 4. Store OFM
    printf("--- Step 4: Configure and Store OFM ---\n");
    printf("Sending CMD_DMA_CONFIG_ADDR for OFM with address: 0x%lx\n", ofm_addr);
    rocc_dma_config_addr(ofm_addr);
    current_status = rocc_get_status();
    printf("Status after CMD_DMA_CONFIG_ADDR for OFM: %lu\n", current_status);

    printf("Sending CMD_DMA_CONFIG_PARAMS for OFM. Length: %lu, Direction: BUF_TO_MEM (%d), BufferID: OFM (%d)\n", ofm_len_bytes, DMA_DIR_BUF_TO_MEM, BUFFER_ID_OFM);
    rocc_dma_config_params(ofm_len_bytes, DMA_DIR_BUF_TO_MEM, BUFFER_ID_OFM);
    current_status = rocc_get_status();
    printf("Status after CMD_DMA_CONFIG_PARAMS for OFM: %lu\n", current_status);
    if (current_status != STATUS_DMA_CONFIG_READY) {
        printf("Error: Expected STATUS_DMA_CONFIG_READY (%d) after OFM params, got %lu\n", STATUS_DMA_CONFIG_READY, current_status);
        return 1;
    }
    printf("OFM DMA configuration successful. Sending CMD_DMA_START for OFM.\n");
    rocc_dma_start();
    current_status = rocc_get_status();
    // printf("Status immediately after CMD_DMA_START for OFM: %lu (Expected: %d for DMA_BUSY)\n", current_status, STATUS_DMA_BUSY);
    if (poll_status_until(STATUS_DMA_OFM_STORE_DONE, STATUS_DMA_ERROR, STATUS_ERROR, "OFM Store") != 0) return 1;
    printf("--- OFM Store Complete ---\n");

    end_cycle = rdcycle_custom();
    printf("End cycle count: %lu\n", end_cycle);

    printf("--- Verification ---\n"); // Modified this line slightly
    printf("Sample OFM data (from accelerator's DMA write to memory):\n");
    for (int i = 0; i < 10 && i < OFM_ROWS * OFM_COLS; ++i) { // Print more samples
        printf("OFM_MEM[%d] = %d\n", i, ofm_data[i]);
    }

    // software_convolution_reference_fixed_point(8); // Calculate expected
    int errors = 0;
    for (int i = 0; i < OFM_ROWS * OFM_COLS; ++i) {
        if (ofm_data[i] != ofm_expected_data[i]) {
            if(errors < 20) { // Print up to 20 mismatches
                printf("Mismatch at OFM[%d]: Expected %d, Got %d\n", i, ofm_expected_data[i], ofm_data[i]);
            }
            errors++;
        }
    }
    if (errors == 0) {
        printf("OFM data matches software reference.\n");
    } else {
        printf("%d mismatches in OFM data compared to software reference.\n", errors);
    }

    printf("CNN Accelerator RoCC Test finished.\n"); // Modified this line slightly
    printf("Total cycles for accelerator operations (approx): %lu\n", end_cycle - start_cycle);
    
    current_status = rocc_get_status();
    printf("Final accelerator status after all operations: %lu\n", current_status);

    if ((current_status == STATUS_IDLE || current_status == STATUS_DMA_OFM_STORE_DONE) && errors == 0) {
          printf("Test PASSED: Operations completed, final status (%lu) is reasonable, and results match.\n", current_status);
    } else if (errors != 0) {
          printf("Test FAILED: Results do not match reference (%d errors).\n", errors);
          return 1;
    }
    else {
          printf("Test FAILED: Final status %lu is unexpected. Expected IDLE (%d) or OFM_STORE_DONE (%d).\n", current_status, STATUS_IDLE, STATUS_DMA_OFM_STORE_DONE);
          return 1;
    }

    return 0;
}