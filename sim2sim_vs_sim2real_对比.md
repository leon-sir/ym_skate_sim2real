# Sim2Sim vs Sim2Real 对比分析

## 核心架构对比

两个文件都遵循相同的控制流程：
```
数据获取 → 观测构建 → RL策略推理 → 动作输出 → 执行器控制
```

---

## 1. 数据获取部分 (Observation)

### Sim2Sim (仿真)
```cpp
// 从 MuJoCo 仿真器获取数据
auto [q_mjc, dq_mjc, quat, projected_gravity, linvel_mjc, angvel_mjc] 
    = get_observation(mj_sim.d, mj_sim.m);

// get_observation() 函数内部：
// - 关节位置/速度: data->qpos, data->qvel
// - 四元数: data->sensordata[orientation_id]
// - 线速度: data->sensordata[linvel_id]
// - 角速度: data->sensordata[angvel_id]
// - 重力投影: 通过四元数计算
```

### Sim2Real (实机)
```cpp
// 从真实硬件获取数据
auto [q_Motor, dq_Motor, quat, acc, angle_vel, projected_gravity] 
    = get_observation();

// get_observation() 函数内部：
// - 关节位置/速度: 从共享内存读取电机数据
//   shm_motor_down.readJointDatafromMotor(recJ);
//   shm_arm_.readJointDatafromMotorArm(recJ_arm);
// - 四元数: data_imu->quat_float[0~3]
// - 加速度: data_imu->accel_float[0~2]
// - 角速度: data_imu->angle_float[0~2]
// - 重力投影: 通过IMU四元数计算
```

**对应关系**:
| 数据类型 | Sim2Sim 来源 | Sim2Real 来源 |
|---------|-------------|--------------|
| 关节位置 q | MuJoCo data->qpos | 电机编码器 (共享内存) |
| 关节速度 dq | MuJoCo data->qvel | 电机速度 (共享内存) |
| 姿态四元数 | MuJoCo sensor | 真实IMU |
| 角速度 | MuJoCo sensor | 真实IMU |
| 重力投影 | 四元数计算 | 四元数计算 |

---

## 2. 观测处理 (完全相同)

两个文件都调用相同的函数：

```cpp
// 关节顺序映射: MuJoCo顺序 → IsaacLab顺序
for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx) {
    size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
    q_lab[lab_idx] = q_mjc[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
    dq_lab[lab_idx] = dq_mjc[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
}
```

---

## 3. RL策略推理 (完全相同)

```cpp
// 两个文件都调用相同的控制器
amp_controller.onnx_output(
    projected_gravity,    // 重力投影
    angvel_mjc,          // 角速度
    velocity_commands,   // 速度命令
    q_lab,              // 关节位置
    dq_lab              // 关节速度
);

// 动作缩放和补偿 (完全相同)
for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i) {
    target_q[i] = amp_controller.actions_[i] * amp_controller.cfg.control.action_scale;
    target_q[i] += amp_controller.cfg.joint_params_isaaclab[i].offset;
    target_q[i] = std::clamp(target_q[i], -amp_controller.cfg.control.clip_actions,
                             amp_controller.cfg.control.clip_actions);
}
```

---

## 4. 动作映射 (完全相同)

```cpp
// IsaacLab顺序 → MuJoCo/电机顺序
for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx) {
    size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
    target_q_mjc[mjc_idx] = target_q[lab_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
    target_dq_mjc[mjc_idx] = target_dq[lab_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
}
```

---

## 5. 执行器控制部分 (不同)

### Sim2Sim (仿真)
```cpp
// 计算力矩
Eigen::VectorXd tau_lab = amp_controller.pd_output(target_q, q_lab, target_dq, dq_lab);

// 限幅
for (size_t i = 0; i < amp_controller.cfg.env.num_joints; ++i) {
    tau_lab[i] = std::clamp(tau_lab[i], 
                           -amp_controller.cfg.joint_params_isaaclab[i].tau_limit,
                            amp_controller.cfg.joint_params_isaaclab[i].tau_limit);
    tau_lab[i] *= amp_controller.cfg.joint_params_isaaclab[i].direction;
}

// 重新排序并发送到 MuJoCo
for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx) {
    size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
    tau_mjc[mjc_idx] = tau_lab[lab_idx];
}

// 发送力矩命令到 MuJoCo
for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i) {
    mj_sim.d->ctrl[i] = tau_mjc[i];  // ← 仿真器接口
}

// 执行仿真步进
mj_step(mj_sim.m, mj_sim.d);
```

### Sim2Real (实机)
```cpp
// 下半身 (腿部): 发送位置+速度+PD参数
for (int i = 0; i < JOINT_MOTOR_NUMBER; ++i) {
    if (i < 13) {  // 腿部关节
        size_t lab_idx = amp_controller.cfg.mjc2lab[i];
        sendDataJoint[i].pos_des_ = target_q_mjc[i];
        sendDataJoint[i].vel_des_ = target_dq_mjc[i];
        sendDataJoint[i].kp_ = amp_controller.cfg.joint_params_isaaclab[lab_idx].kp;
        sendDataJoint[i].kd_ = amp_controller.cfg.joint_params_isaaclab[lab_idx].kd;
        sendDataJoint[i].ff_ = 0.0;
    }
}
// 发送到腿部电机
shm_motor_down.writeJointDatatoMotor(sendDataJoint);  // ← 电机接口

// 上半身 (手臂): 只发送位置
for (int i = 0; i < JOINT_ARM_NUMBER; i++) {
    if (i < 10) {
        pos_des_arm_[i] = target_q_mjc[i + 13];
    }
}
// 发送到手臂电机
shm_arm_.writeJointDatatoMotorArm(pos_des_arm_);  // ← 电机接口
```

**关键区别**:
| 项目 | Sim2Sim | Sim2Real |
|-----|---------|----------|
| 控制量 | 力矩 (tau) | 位置+速度+PD参数 |
| 接口 | `mj_sim.d->ctrl[i]` | 共享内存 (电机驱动) |
| PD控制 | 在代码中计算力矩 | 在电机驱动器中执行 |
| 执行 | `mj_step()` 仿真步进 | 电机实时响应 |

---

## 6. 控制频率

### Sim2Sim
```cpp
// 可调节的仿真速度
double speed_factor = 1;
double screen_refresh_rate = 60.0;
double steps = 1.6 / screen_refresh_rate / mj_sim.m->opt.timestep * speed_factor;

// 策略推理频率: 每 decimation 步 (默认10步)
if (count_onnx % sim_cfg.decimation == 0) {
    amp_controller.onnx_output(...);  // 100Hz
}
```

### Sim2Real
```cpp
// 固定的实时控制周期
auto cycle_duration = milliseconds(static_cast<int>(real_cfg.dt * 1000));  // 10ms
auto next_time_point_motor = std::chrono::steady_clock::now() + cycle_duration;

// 策略推理频率: 每 decimation 次 (默认1次)
if (count_onnx % real_cfg.decimation == 0) {
    amp_controller.onnx_output(...);  // 100Hz
}

// 严格的实时循环
std::this_thread::sleep_until(next_time_point_motor);
next_time_point_motor += cycle_duration;
```

---

## 7. 模式切换

### Sim2Sim
```cpp
if (count_pd < time_pd / mj_sim.m->opt.timestep) {
    // PD站立模式
} else {
    // RL行走模式
}
```

### Sim2Real
```cpp
if (loadControllerFlag_ && !emergencyStopFlag_) {
    if (setWalkFlag_ && !standingModeFlag) {
        // RL行走模式
    } else {
        // PD站立模式
    }
} else {
    // 安全模式 (所有电机零力矩)
}
```

---

## 总结：统一的接口设计

```
┌─────────────────────────────────────────────────────────┐
│              AMPController (RL策略核心)                   │
│  - onnx_output(): 观测 → 动作                            │
│  - pd_output(): 动作 → 力矩 (仅sim2sim使用)              │
└─────────────────────────────────────────────────────────┘
                          ↓
        ┌─────────────────┴─────────────────┐
        ↓                                   ↓
┌───────────────────┐            ┌──────────────────────┐
│   Sim2Sim         │            │   Sim2Real           │
├───────────────────┤            ├──────────────────────┤
│ 输入: MuJoCo传感器 │            │ 输入: IMU + 电机编码器 │
│ 输出: 力矩命令     │            │ 输出: 位置+PD参数     │
│ 执行: mj_step()   │            │ 执行: 共享内存通信    │
└───────────────────┘            └──────────────────────┘
```

**核心思想**: 
- **相同的大脑** (AMPController)
- **不同的身体** (MuJoCo vs 真实电机)
- **统一的接口** (观测向量 → 动作向量)

这种设计让你可以在仿真中验证策略，然后无缝迁移到实机！
