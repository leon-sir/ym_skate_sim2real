import pandas as pd
import matplotlib.pyplot as plt

# === 1. 读取两个文件 ===
file1 = "/home/pc/ymbot_e/src/data/sim2real_rec_joint.txt"  # 发的电机数据
file2 = "/home/pc/ymbot_e/src/data/sim2real_send_joint.txt"  # 反馈电机数据

# 假设逗号分隔，没有表头
df1 = pd.read_csv(file1, header=None)
df2 = pd.read_csv(file2, header=None)

# 确保列数一致
assert df1.shape[1] == df2.shape[1], "两个文件的列数不一致！"
num_cols = df1.shape[1]
print(f"两个文件都有 {num_cols} 列")

# === 2. 逐列绘制对比图 ===
for col in range(num_cols):
    col1 = df1.iloc[:, col]  # 发的
    col2 = df2.iloc[:, col]  # 反馈
    diff = col2 - col1       # 差值

    plt.figure(figsize=(12, 5))

    # 发的 vs 反馈
    plt.plot(col1.index, col1, label="发的 (File1)", color="blue")
    plt.plot(col2.index, col2, label="反馈 (File2)", color="red", linestyle="--")

    # 差值
    plt.plot(diff.index, diff, label="反馈-发的", color="green", alpha=0.7)

    plt.xlabel("行号 (时间序列)")
    plt.ylabel("电机数值")
    plt.title(f"第 {col+1} 列：发的 vs 反馈 vs 差值")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # 打印误差统计
    print(f"=== 第 {col+1} 列误差统计 ===")
    print(f"平均误差: {diff.mean():.6f}")
    print(f"最大误差: {diff.max():.6f}")
    print(f"最小误差: {diff.min():.6f}")
    print(f"均方根误差(RMSE): {(diff**2).mean()**0.5:.6f}")
    print()
