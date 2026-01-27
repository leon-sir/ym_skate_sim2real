#!/bin/bash

# 实时日志记录脚本
# 用法: ./start_logging.sh [your_program_command]

# 创建logs目录
mkdir -p logs

# 生成时间戳
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="logs/rl_observations_${TIMESTAMP}.log"
ACTION_LOG="logs/action_analysis_${TIMESTAMP}.log"

echo "开始记录日志..."
echo "主日志文件: $LOG_FILE"
echo "Action分析文件: $ACTION_LOG"
echo "按 Ctrl+C 停止记录"

# 如果提供了命令参数，执行该命令并记录日志
if [ $# -gt 0 ]; then
    echo "执行命令: $@"
    echo "执行命令: $@" > "$LOG_FILE"
    echo "执行命令: $@" > "$ACTION_LOG"
    
    # 使用tee同时输出到终端和文件，并用grep过滤action相关信息
    "$@" 2>&1 | tee >(cat >> "$LOG_FILE") | tee >(grep -i "action\|warning\|onnx" >> "$ACTION_LOG")
else
    echo "请提供要执行的命令，例如:"
    echo "./start_logging.sh roslaunch your_package your_launch_file.launch"
    echo ""
    echo "或者手动输入日志内容 (Ctrl+D 结束):"
    
    # 从标准输入读取并记录
    while IFS= read -r line; do
        echo "$(date '+%Y-%m-%d %H:%M:%S.%3N') $line" | tee -a "$LOG_FILE"
        
        # 如果包含action相关关键词，也写入action日志
        if echo "$line" | grep -qi "action\|warning\|onnx"; then
            echo "$(date '+%Y-%m-%d %H:%M:%S.%3N') $line" >> "$ACTION_LOG"
        fi
    done
fi

echo ""
echo "日志记录完成!"
echo "主日志文件: $LOG_FILE"
echo "Action分析文件: $ACTION_LOG"