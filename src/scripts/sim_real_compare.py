import numpy as np
import matplotlib.pyplot as plt

# 读取 txt 文件
def load_data(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        data = [list(map(float, line.strip().split(','))) for line in lines]
    return np.array(data)

# 加载两个文件的数据
# data1 = load_data('/home/pc/ymbot_e/src/data/sim2real_rec_joint.txt')
# data2 = load_data('/home/pc/ymbot_e/src/data/sim2real_send_joint.txt')
# data1 = load_data('/home/pc/ymbot_e/src/data/sim2real_onnx_rec_joint.txt')
data2 = load_data('/home/pc/ymbot_e/src/data/sim2real_onnx_send_joint.txt')
# data2 = load_data('/home/pc/ymbot_e/src/data/sim2sim_onnx_rec_joint.txt')
data1 = load_data('/home/pc/ymbot_e/src/data/sim2sim_onnx_send_joint.txt')
print(data1.shape)
print(data2.shape)


# 对齐到相同的最短行数
min_len = min(data1.shape[0], data2.shape[0])
data1 = data1[:min_len, :]
data2 = data2[:min_len, :]
print(f"对齐后 shape: {data1.shape}")

# 时间轴
time = data1[:, 0]

# 创建 22 个子图
fig, axs = plt.subplots(11, 2, figsize=(15, 20))  # 11行2列的子图
axs = axs.flatten()

for i in range(22):
    col_idx = i + 1  # 数据从第2列开始（索引1）
    axs[i].plot(time, data1[:, col_idx], label='sim ')
    axs[i].plot(time, data2[:, col_idx], label='real ')
    axs[i].set_title(f'Data Column {i+1}')
    axs[i].legend()
    axs[i].grid(True)

plt.tight_layout()
plt.show()
