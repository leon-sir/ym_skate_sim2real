#!/bin/bash
mkdir -p /home/pc/tmp/

# 清空之前的日志文件
rm -f /home/pc/tmp/*.log
echo "已清空之前的日志文件"

echo "----start process----"
echo "------启动控制程序------"
echo "----Warning: Plese wait at least 3 minutes before operating the remote control----"
echo "-----------------------警告：请至少等待三分钟再操纵遥控器-------------------------------"

# 获取当前终端
current_terminal=$(tty)

# 获取当前用户
current_user=$(whoami)

# 关闭除当前终端外的其他终端（避免误杀关键进程）
for terminal in $(ps -eo pid,tty,comm,user | grep -E 'tty|pts' | grep -v "?" |grep -v "$current_terminal" | grep -v "$current_user" | awk '{print $2}'); do
    echo "Processing terminal: $terminal"

    # 获取该终端中的所有进程PID
    for pid in $(ps -eo pid,tty | grep "$terminal" | awk '{print $1}'); do
        if [ ! -z "$pid" ] && [ "$terminal" != "?" ]; then
            echo "Terminating process $pid on terminal $terminal"
            # 向进程发送SIGTERM信号，优雅地终止进程
            sudo kill -SIGTERM $pid
            sleep 5  # 给进程一点时间完成退出
        fi
    done

    # 确保进程被结束后，再强制关闭剩余进程（防止未响应的情况）
    for pid in $(ps -eo pid,tty | grep "$terminal" | awk '{print $1}'); do
        if [ ! -z "$pid" ]; then
            echo "Force killing process $pid on terminal $terminal"
            sudo kill -9 $pid
        fi
    done
done

# 等待5秒，检查是否还有终端打开
sleep 2

# 检查是否有其他终端在运行
if [ $(ps -eo pid,tty,comm | grep -E 'tty|pts' | grep -v "$current_terminal" | wc -l) -eq 0 ]; then
    echo "No other terminals open. Skipping terminal check."
else
    echo "Other terminals are still open."
fi

# 进入项目文件
echo "start the first step: >>>>>20%"
gnome-terminal -- bash -c "

cd ~/ymbot_e_13dof_skate;
# 进入root权限并设置ROS环境
sudo su -c 'source devel/setup.bash && roslaunch /home/pc/ymbot_e_13dof_skate/src/rl_additional/yesense/launch/run_without_rviz.launch' ;
exec bash"

# 等待2秒钟
echo "plese wait"
sleep 2

# 打开新终端并执行VRPN重力投影节点
echo "start the second step:>>>>40%"
gnome-terminal -- bash -c "
# 定义通知函数
send_notification() {
    local title=\"\$1\"
    local message=\"\$2\"
    local client_id=\"\${3:-3}\"
    
    curl -X POST \"https://goty.bismih.cn/message?token=AT6z4tx3V6-2j.Q\" \
      -H \"Content-Type: application/json\" \
      -d \"{
        \\\"title\\\": \\\"\$title\\\",
        \\\"message\\\": \\\"\$message\\\",
        \\\"priority\\\": 5,
        \\\"extras\\\": {
          \\\"client::client\\\": {
            \\\"id\\\": \$client_id
          }
        }
      }\" >/dev/null 2>&1 &
}

# 监控函数
monitor_output() {
    while IFS= read -r line; do
        timestamp=\$(date '+[%Y-%m-%d %H:%M:%S]')
        echo \"\$timestamp \$line\" | tee -a \"/home/pc/tmp/vrpn_gravity_projection_\$(date +%F).log\"
        
        if echo \"\$line\" | grep -E -q 'error|fail|exception|connection lost|timeout'; then
            send_notification \"VRPN重力投影警报\" \"检测到问题: \$line\" 3
        fi
    done
}

cd ~/vrpn_ws;
source devel/setup.bash && roslaunch vrpn_gravity_projection reset_four_element.launch 2>&1 | monitor_output;
exec bash"

# 等待3秒钟
echo "plese wait"
sleep 3

# 打开新终端并执行第三个roslaunch命令
echo "start the third step:>>>>>>>>>>60%"
gnome-terminal -- bash -c "
# 定义通知函数
send_notification() {
    local title=\"\$1\"
    local message=\"\$2\"
    local client_id=\"\${3:-3}\"
    
    curl -X POST \"https://goty.bismih.cn/message?token=AT6z4tx3V6-2j.Q\" \
      -H \"Content-Type: application/json\" \
      -d \"{
        \\\"title\\\": \\\"\$title\\\",
        \\\"message\\\": \\\"\$message\\\",
        \\\"priority\\\": 5,
        \\\"extras\\\": {
          \\\"client::client\\\": {
            \\\"id\\\": \$client_id
          }
        }
      }\" >/dev/null 2>&1 &
}

# 监控函数
monitor_output() {
    while IFS= read -r line; do
        timestamp=\$(date '+[%Y-%m-%d %H:%M:%S]')
        echo \"\$timestamp \$line\" | tee -a \"/home/pc/tmp/encos_motor_interface_\$(date +%F).log\"
        
        if echo \"\$line\" | grep -E -q 'error|fail|exception|motor fault|connection lost'; then
            send_notification \"因克斯电机接口警报\" \"检测到问题: \$line\" 3
        fi
    done
}

cd ~/ymbot_e_13dof_skate;
sudo su -c 'source devel/setup.bash && rosrun ymbot_amp_devel encos_motor_interface 2>&1' | monitor_output;
exec bash"

# 等待5秒钟
echo "plese wait"
sleep 5

# 打开新终端并执行第四个roslaunch命令
echo "start the fourth step:>>>>>>>>>>>>>>>80%"
gnome-terminal -- bash -c "
# 定义通知函数
send_notification() {
    local title=\"\$1\"
    local message=\"\$2\"
    local client_id=\"\${3:-3}\"
    
    curl -X POST \"https://goty.bismih.cn/message?token=AT6z4tx3V6-2j.Q\" \
      -H \"Content-Type: application/json\" \
      -d \"{
        \\\"title\\\": \\\"\$title\\\",
        \\\"message\\\": \\\"\$message\\\",
        \\\"priority\\\": 5,
        \\\"extras\\\": {
          \\\"client::client\\\": {
            \\\"id\\\": \$client_id
          }
        }
      }\" >/dev/null 2>&1 &
}

# 监控函数
monitor_output() {
    while IFS= read -r line; do
        timestamp=\$(date '+[%Y-%m-%d %H:%M:%S]')
        echo \"\$timestamp \$line\" | tee -a \"/home/pc/tmp/eumotor_interface_amp_\$(date +%F).log\"
        
        if echo \"\$line\" | grep -E -q 'error from eu_canable.dll|write failed'; then
            send_notification \"EU电机接口警报\" \"检测到问题: \$line\" 3
        fi
    done
}

cd ~/ymbot_e_13dof_skate;
bash -c 'source devel/setup.bash && rosrun ymbot_amp_devel eumotor_interface_amp 2>&1' | monitor_output;
exec bash"

# 等待10秒钟
echo "plese wait"
sleep 10

# 打开新终端并执行第五个roslaunch命令
echo "start the fifth step:>>>>>>>>>>>>>>>>>>>>100%"
gnome-terminal -- bash -c "
# 定义通知函数
send_notification() {
    local title=\"\$1\"
    local message=\"\$2\"
    local client_id=\"\${3:-3}\"
    
    curl -X POST \"https://goty.bismih.cn/message?token=AT6z4tx3V6-2j.Q\" \
      -H \"Content-Type: application/json\" \
      -d \"{
        \\\"title\\\": \\\"\$title\\\",
        \\\"message\\\": \\\"\$message\\\",
        \\\"priority\\\": 5,
        \\\"extras\\\": {
          \\\"client::client\\\": {
            \\\"id\\\": \$client_id
          }
        }
      }\" >/dev/null 2>&1 &
}

# 监控函数
monitor_output() {
    while IFS= read -r line; do
        timestamp=\$(date '+[%Y-%m-%d %H:%M:%S]')
        echo \"\$timestamp \$line\" | tee -a \"/home/pc/tmp/ymbot_e_rl_sim2real_\$(date +%F).log\"
        
        if echo \"\$line\" | grep -E -q 'error|fail|exception|crash|segmentation|timeout|sim2real failed'; then
            send_notification \"仿真到现实节点警报\" \"检测到问题: \$line\" 3
        elif echo \"\$line\" | grep -E -q 'startup complete|system ready|initialization successful'; then
            send_notification \"系统启动成功\" \"仿真到现实节点已成功启动: \$line\" 3
        fi
    done
}

cd ~/ymbot_e_13dof_skate;
sudo su -c 'source devel/setup.bash && roslaunch /home/pc/ymbot_e_13dof_skate/src/ymbot_amp_devel/launch/skate_amp_rl_sim2real.launch 2>&1' | monitor_output;
exec bash"

echo "Script completed successfully."
echo "脚本运行成功."

# 发送启动完成通知
curl -X POST "https://goty.bismih.cn/message?token=AT6z4tx3V6-2j.Q" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "系统启动完成",
    "message": "所有控制节点已启动，请等待3分钟后再操作遥控器",
    "priority": 5,
    "extras": {
      "client::client": {
        "id": 3
      }
    }
  }' >/dev/null 2>&1 &

# 等待所有进程
wait
