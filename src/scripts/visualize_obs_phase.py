#!/usr/bin/env python3
"""
可视化obs_log.txt中的phase观测数据 (obs_[51]-obs_[56])
提取每个时间点的phase值并绘制时间序列图
"""

import re
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse

def parse_timestamp(timestamp_str):
    """解析时间戳字符串，返回相对于起始时间的秒数"""
    # 格式: [2026-02-10 16:33:18.136]
    timestamp_pattern = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]'
    match = re.search(timestamp_pattern, timestamp_str)
    if match:
        time_str = match.group(1)
        dt = datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S.%f')
        return dt
    return None

def extract_phase_data(log_file_path):
    """
    从日志文件中提取obs_[51]-obs_[56]的数据
    返回: timestamps, phase_data字典
    """
    # 正则表达式匹配phase观测行
    phase_patterns = {
        51: r'obs_\[51\] = phase\[0\] = ([+-]?[0-9]*\.?[0-9]+([eE][+-]?[0-9]+)?)',
        52: r'obs_\[52\] = phase\[1\] = ([+-]?[0-9]*\.?[0-9]+([eE][+-]?[0-9]+)?)',
        53: r'obs_\[53\] = phase\[2\] = ([+-]?[0-9]*\.?[0-9]+([eE][+-]?[0-9]+)?)',
        54: r'obs_\[54\] = phase\[3\] = ([+-]?[0-9]*\.?[0-9]+([eE][+-]?[0-9]+)?)',
        55: r'obs_\[55\] = phase\[4\] = ([+-]?[0-9]*\.?[0-9]+([eE][+-]?[0-9]+)?)',
        56: r'obs_\[56\] = phase\[5\] = ([+-]?[0-9]*\.?[0-9]+([eE][+-]?[0-9]+)?)'
    }
    
    # 存储数据
    timestamps = []
    phase_data = {i: [] for i in range(51, 57)}
    current_timestamp = None
    
    print(f"正在读取文件: {log_file_path}")
    
    with open(log_file_path, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
                
            # 检查是否有新的时间戳
            ts = parse_timestamp(line)
            if ts:
                current_timestamp = ts
            
            # 检查是否包含phase数据
            for obs_index, pattern in phase_patterns.items():
                match = re.search(pattern, line)
                if match and current_timestamp:
                    value = float(match.group(1))
                    # 只有当时间戳改变时才添加新数据点
                    if not timestamps or current_timestamp != timestamps[-1]:
                        timestamps.append(current_timestamp)
                        # 为所有phase变量添加当前值
                        for idx in range(51, 57):
                            if idx == obs_index:
                                phase_data[idx].append(value)
                            else:
                                # 如果不是当前匹配的索引，使用上一个值或0
                                if phase_data[idx]:
                                    phase_data[idx].append(phase_data[idx][-1])
                                else:
                                    phase_data[idx].append(0.0)
                    else:
                        # 更新同一时间戳下的值
                        if phase_data[obs_index]:
                            phase_data[obs_index][-1] = value
    
    print(f"共找到 {len(timestamps)} 个时间点的数据")
    return timestamps, phase_data


def plot_phase_data(timestamps, phase_data, save_path=None):
    """
    绘制phase数据的时间序列图
    """
    if not timestamps:
        print("没有找到数据!")
        return
    
    # 转换时间为相对秒数
    start_time = timestamps[0]
    time_seconds = [(ts - start_time).total_seconds() for ts in timestamps]
    
    # 创建图形
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle('Phase Observations (obs_[51]-obs_[56])', fontsize=16)
    
    # phase索引到子图位置的映射
    subplot_map = {51: (0,0), 52: (0,1), 53: (1,0), 54: (1,1), 55: (2,0), 56: (2,1)}
    
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
    
    for obs_index, (row, col) in subplot_map.items():
        ax = axes[row, col]
        data = phase_data[obs_index]
        
        if data:
            ax.plot(time_seconds, data, 
                   color=colors[obs_index-51], 
                   linewidth=1.5,
                   marker='o', 
                   markersize=2,
                   label=f'phase[{obs_index-51}]')
            
            ax.set_xlabel('Time (seconds)')
            ax.set_ylabel('Phase Value')
            ax.set_title(f'obs_[{obs_index}] = phase[{obs_index-51}]')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            # 设置y轴范围为[0,1]，因为phase通常是归一化值
            ax.set_ylim(-0.1, 1.1)
            # ax.set_xlim(20, 30)
    
    plt.tight_layout()
    
    # 显示最终读数
    print("\n=== 最终读数 ===")
    for obs_index in range(51, 57):
        if phase_data[obs_index]:
            final_value = phase_data[obs_index][-1]
            print(f"obs_[{obs_index}] = phase[{obs_index-51}] = {final_value}")
    
    print(f"\n总时长: {time_seconds[-1]:.2f} 秒")
    print(f"数据点数: {len(time_seconds)}")
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n图表已保存到: {save_path}")
    
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='可视化obs_log.txt中的phase观测数据')
    parser.add_argument('--log-file', '-f', 
                       default='/home/ps/pan_zheng_proj/ym_skate_sim2real/obs_log.txt',
                       help='日志文件路径')
    parser.add_argument('--save', '-s',
                       help='保存图片的路径')
    
    args = parser.parse_args()
    
    try:
        # 提取数据
        timestamps, phase_data = extract_phase_data(args.log_file)
        
        # 绘制图表
        plot_phase_data(timestamps, phase_data, args.save)
        
    except FileNotFoundError:
        print(f"错误: 找不到文件 {args.log_file}")
    except Exception as e:
        print(f"处理过程中出现错误: {e}")

if __name__ == "__main__":
    main()