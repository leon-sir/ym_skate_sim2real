#!/usr/bin/env python3
"""
实时日志捕获脚本
用于捕获RL观测数据和ONNX推理日志并写入文件
"""

import subprocess
import datetime
import os
import sys
import signal
import threading
from pathlib import Path

class LogCapture:
    def __init__(self, log_dir="logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        
        # 创建带时间戳的日志文件
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"rl_observations_{timestamp}.log"
        self.action_log_file = self.log_dir / f"action_analysis_{timestamp}.log"
        
        self.running = True
        self.log_handle = None
        self.action_log_handle = None
        
    def start_logging(self, command=None):
        """开始日志记录"""
        try:
            # 打开日志文件
            self.log_handle = open(self.log_file, 'w', buffering=1)  # 行缓冲
            self.action_log_handle = open(self.action_log_file, 'w', buffering=1)
            
            print(f"开始记录日志到: {self.log_file}")
            print(f"Action分析日志: {self.action_log_file}")
            
            if command:
                # 如果提供了命令，启动子进程并捕获输出
                self.capture_subprocess_output(command)
            else:
                # 否则从stdin读取
                self.capture_stdin()
                
        except KeyboardInterrupt:
            print("\n停止日志记录...")
        finally:
            self.cleanup()
    
    def capture_subprocess_output(self, command):
        """捕获子进程输出"""
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            while self.running and process.poll() is None:
                line = process.stdout.readline()
                if line:
                    self.process_line(line.strip())
                    
        except Exception as e:
            print(f"捕获子进程输出时出错: {e}")
    
    def capture_stdin(self):
        """从标准输入捕获日志"""
        print("请粘贴日志内容 (Ctrl+C 停止):")
        try:
            while self.running:
                line = input()
                self.process_line(line)
        except EOFError:
            pass
    
    def process_line(self, line):
        """处理每一行日志"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        # 写入主日志文件
        self.log_handle.write(f"[{timestamp}] {line}\n")
        
        # 分析action相关信息
        if "action:" in line.lower():
            self.analyze_action_line(line, timestamp)
        elif "warning: action" in line.lower():
            self.analyze_warning_line(line, timestamp)
        elif "onnx:" in line.lower():
            self.analyze_onnx_line(line, timestamp)
        
        # 实时显示重要信息
        if any(keyword in line.lower() for keyword in ["action:", "warning:", "onnx:"]):
            print(f"[{timestamp}] {line}")
    
    def analyze_action_line(self, line, timestamp):
        """分析action行"""
        self.action_log_handle.write(f"[{timestamp}] ACTION_LINE: {line}\n")
        
        # 检查是否所有action都为0
        if "action:" in line.lower():
            # 提取action数组
            try:
                action_part = line.split("action:")[1].strip()
                if "[0.0000, 0.0000, 0.0000" in action_part:
                    self.action_log_handle.write(f"[{timestamp}] WARNING: All actions are ZERO!\n")
            except:
                pass
    
    def analyze_warning_line(self, line, timestamp):
        """分析警告行"""
        self.action_log_handle.write(f"[{timestamp}] CLAMP_WARNING: {line}\n")
        
        # 提取被截断的action信息
        if "clamped from" in line:
            try:
                parts = line.split("clamped from")
                if len(parts) > 1:
                    original_value = parts[1].split("to")[0].strip()
                    clamped_value = parts[1].split("to")[1].strip()
                    action_index = line.split("Action[")[1].split("]")[0]
                    
                    self.action_log_handle.write(
                        f"[{timestamp}] ACTION_CLAMP: Index={action_index}, "
                        f"Original={original_value}, Clamped={clamped_value}\n"
                    )
            except:
                pass
    
    def analyze_onnx_line(self, line, timestamp):
        """分析ONNX推理信息"""
        if "inference completed" in line.lower():
            self.action_log_handle.write(f"[{timestamp}] ONNX_INFERENCE: {line}\n")
    
    def cleanup(self):
        """清理资源"""
        self.running = False
        if self.log_handle:
            self.log_handle.close()
        if self.action_log_handle:
            self.action_log_handle.close()
        print(f"\n日志已保存到: {self.log_file}")
        print(f"Action分析已保存到: {self.action_log_file}")

def signal_handler(signum, frame):
    """信号处理器"""
    print("\n接收到停止信号...")
    sys.exit(0)

def main():
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 创建日志捕获器
    logger = LogCapture()
    
    if len(sys.argv) > 1:
        # 如果提供了命令参数，执行该命令并捕获输出
        command = sys.argv[1:]
        logger.start_logging(command)
    else:
        # 否则从stdin读取
        logger.start_logging()

if __name__ == "__main__":
    main()