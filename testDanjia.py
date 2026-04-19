import math
import xintrans
import os

data = [['90310', (-0.45, -0.45, 0.0)], ['90311', (0.45, -0.45, 0.0)], ['90910', (-0.488, -0.488, 0.721)], ['90911', (0.488, -0.488, 0.721)], ['90320', (-0.59, -0.59, 2.666)],
        ['90321', (0.59, -0.59, 2.666)], ['101010', (-0.643, -0.643, 3.652)], ['101011', (0.643, -0.643, 3.652)], ['100320', (-0.787, -0.787, 6.342)], ['100321', (0.787, -0.787, 6.342)],
        ['111210', (-0.843, -0.843, 7.364)], ['111211', (0.843, -0.843, 7.364)], ['110320', (-0.982, -0.982, 9.928)], ['110321', (0.982, -0.982, 9.928)], ['121510', (-1.031, -1.031, 10.843)], ['121511', (1.031, -1.031, 10.843)]]

jiedian, ganjian = xintrans.work(r"D:\SanWei\zuobiao", data)
print(jiedian)
print(ganjian)


output_dir = r"D:\SanWei\result"
os.makedirs(output_dir, exist_ok=True)
output_file = os.path.join(output_dir, "result.txt")

with open(output_file, 'w', encoding='utf-8') as f:
    f.write("=================== 担架部分生成的节点 (jiedian) ===================\n\n")
    for node in jiedian:
        f.write(str(node) + "\n\n")

    f.write("=================== 担架部分生成的杆件 (ganjian) ===================\n\n")
    for rod in ganjian:
        f.write(str(rod) + "\n\n")

print(f"共生成 {len(jiedian)} 个节点，{len(ganjian)} 根杆件")
