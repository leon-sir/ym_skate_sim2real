import numpy as np
import matplotlib.pyplot as plt

# 读取 txt 文件
def load_data(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        data = [list(map(float, line.strip().split(','))) for line in lines]
    return np.array(data)

# 加载两个文件的数据
data1 = load_data('/home/pc/ymbot_e/src/data/sim2real_rec_velocity.txt')
print(data1.shape)


# 时间轴
time = data1[:, 0]

# 创建 22 个子图
fig, axs = plt.subplots(11, 2, figsize=(15, 20))  # 11行2列的子图
axs = axs.flatten()

for i in range(22):
    col_idx = i + 1  # 数据从第2列开始（索引1）
    axs[i].plot(time, data1[:, col_idx], label='recv_velocity')
    axs[i].set_title(f'Data Column {i+1}')
    axs[i].legend()
    axs[i].grid(True)

plt.tight_layout()
plt.show()
