import matplotlib.pyplot as plt
import numpy as np

# 读取数据文件
file_path = '/home/pc/ymbot_e_13dof_skate/dataMotor.txt'

# ---------------- 修复部分：自动清理空行、空值、无效数据 ----------------
# 逐行读取并过滤无效数据
with open(file_path, 'r') as f:
    lines = [line.strip() for line in f if line.strip()]  # 去掉空行

# 按逗号分割，过滤空字符串，再转浮点数
clean_data = []
for line in lines:
    parts = line.split(',')
    # 过滤空字符串、空白字符
    valid_parts = [p.strip() for p in parts if p.strip()]
    if valid_parts:
        clean_data.append([float(p) for p in valid_parts])

data = np.array(clean_data)
# ----------------------------------------------------------------------

# 自动计算关节数量 (总列数-1)/6
num_cols = data.shape[1]
joint_count = (num_cols - 1) // 6

# 验证计算结果
if (num_cols - 1) % 6 != 0:
    raise ValueError(f"Invalid data format: expected columns=1+6*N, got {num_cols}")

# 提取时间数据和关节力矩数据
# 关节力矩位于最后JOINT_MOTOR_NUM列 (索引范围: 5*joint_count+1 到末尾)
time = data[:, 0]
joint_torque = data[:, 5*joint_count+1:]

# 创建绘图
plt.figure(figsize=(12, 8))
for i in range(joint_count):
    plt.plot(time, joint_torque[:, i], label=f'Joint {i+1}')

plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Torque (Nm)', fontsize=12)
plt.title('Joint Torque Profile', fontsize=14)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(loc='best', ncol=2)
plt.tight_layout()

# 显示或保存结果
plt.savefig('/home/pc/ymbot_e_13dof_skate/joint_torque_plot.png', dpi=300)
print(f'Plot saved as joint_torque_plot.png (showing {joint_count} joints)')
plt.show()