# Observation Logging 功能说明

## 修改内容

已将 `amp_controller.cpp` 中的观测数据（obs）从终端打印改为实时写入log文件。

## 修改的文件

1. **amp_controller.hpp**
   - 添加了 `#include <fstream>` 头文件
   - 添加了成员变量 `std::ofstream obs_log_file_;` 用于文件写入

2. **amp_controller.cpp**
   - 添加了 `#include <ctime>` 头文件
   - 在构造函数中打开log文件：`/home/pc/ymbot_e_13dof_skate/obs_log.txt`
   - 在 `onnx_output()` 函数中将所有obs相关的调试信息从 `std::cerr` 改为写入 `obs_log_file_`

## Log文件位置

```
/home/pc/ymbot_e_13dof_skate/obs_log.txt
```

## Log内容

Log文件会记录：
- 前3次调用的详细obs构建过程
- 包括：
  - base_linear_velocity (线速度)
  - base_angular_velocity (角速度)
  - projected_gravity (投影重力)
  - velocity_commands (速度命令)
  - joint_pos (关节位置，带offset校正)
  - joint_vel (关节速度，带缩放)
  - actions (上一次的动作)
  - phase (相位信息)
  - clipping前后的obs统计信息

## 特点

- **实时写入**：每次调用 `onnx_output()` 后立即 `flush()` 确保数据实时写入
- **追加模式**：使用 `std::ios::app` 模式，不会覆盖之前的数据
- **自动时间戳**：文件开头会记录启动时间
- **性能优化**：只记录前3次详细调用，避免log文件过大

## 使用方法

1. 编译项目
2. 运行程序
3. 查看log文件：`cat /home/pc/ymbot_e_13dof_skate/obs_log.txt`
4. 实时监控：`tail -f /home/pc/ymbot_e_13dof_skate/obs_log.txt`

## 清理log文件

如果需要清空log文件：
```bash
> /home/pc/ymbot_e_13dof_skate/obs_log.txt
```

或删除：
```bash
rm /home/pc/ymbot_e_13dof_skate/obs_log.txt
```
