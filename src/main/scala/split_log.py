input_file =  "/home/lzy/chipyard/sims/verilator/output/chipyard.harness.TestHarness.MyCNNConfig/test.out"
output_file = "/home/lzy/chipyard/sims/verilator/output/chipyard.harness.TestHarness.MyCNNConfig/rocc.txt"
import os

# 读取这个文件，把以RoCC开头的行提取出来，写入到一个新的文件中
with open(input_file, 'r') as file:
    lines = file.readlines()
    # 提取以RoCC开头的行
    rocc_lines = [line for line in lines if line.startswith("RoCC Cycle")]
    # 将提取的行写入到新的文件中
with open(output_file, 'w') as output:
    output.writelines(rocc_lines)

