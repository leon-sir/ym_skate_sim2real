import numpy as np
import matplotlib.pyplot as plt

# 读取 txt 文件
def load_data(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        data = [list(map(float, line.strip().split(','))) for line in lines]
    return np.array(data)

# 加载两个文件的数据
data = load_data('/home/pc/ymbot_e/src/data/sim2real_rec_velocity.txt')
print(data.shape)


# 分别取出投影列
x_values = data[:, 1]  # 第二列
y_values = data[:, 2]  # 第三列
z_values = data[:, 3]  # 第四列
# 时间轴
time = data[:, 0]


# 定义参考值
ref_x = 0
ref_y = 0
ref_z = -1

# 计算每个方向的误差（绝对值）
error_x = np.abs(x_values - ref_x)
error_y = np.abs(y_values - ref_y)
error_z = np.abs(z_values - ref_z)

# 计算平均误差
mean_error_x = np.mean(error_x)
mean_error_y = np.mean(error_y)
mean_error_z = np.mean(error_z)

print(f"X方向平均误差: {mean_error_x:.6f}")
print(f"Y方向平均误差: {mean_error_y:.6f}")
print(f"Z方向平均误差: {mean_error_z:.6f}")


# 创建 3 个子图
fig, axs = plt.subplots(3, 1, figsize=(15, 20))  # 11行2列的子图
axs = axs.flatten()

for i in range(3):
    col_idx = i + 1  # 数据从第2列开始（索引1）
    axs[i].plot(time, data[:, col_idx], label='projected_gravity')
    axs[i].set_title(f'Data Column {i+1}')
    axs[i].legend()
    axs[i].grid(True)

plt.tight_layout()
plt.show()