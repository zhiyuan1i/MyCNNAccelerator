input_file =  "/home/lzy/chipyard/sims/verilator/output/chipyard.harness.TestHarness.MyCNNConfig/test.out"
output_file = "/home/lzy/chipyard/sims/verilator/output/chipyard.harness.TestHarness.MyCNNConfig/rocc.txt"
import os

# 读取这个文件，把以RoCC开头的行提取出来，写入到一个新的文件中
with open(input_file, 'r') as file:
    lines = file.readlines()
    prefixes_for_ofm_debug = [
        "ROCC_CTRL: DMA reading OFM_BUF", # Shows data RoCC provides to DMA for OFM
        "DMA_BEAT_ASSM Cycle",            # Shows how the first OFM beat is assembled in DMA
        "DMA_PUT_PREP Cycle",             # Shows the first OFM beat data just before TileLink Put
        "DMA_PUT_FIRE Cycle"              # Confirms the first OFM beat Put operation fired
    ]

    # Modify your line to use these prefixes:
    rocc_lines = [
        line for line in lines if any(line.strip().startswith(prefix) for prefix in prefixes_for_ofm_debug)
    ]
    # 将提取的行写入到新的文件中
with open(output_file, 'w') as output:
    output.writelines(rocc_lines)

