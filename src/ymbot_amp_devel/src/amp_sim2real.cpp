 #include "ymbot_amp_devel/SharedMemory.hpp"
#include "ymbot_amp_devel/SharedMemoryArm.hpp"
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <algorithm> // for std::clamp
#include <cmath>
#include <deque>
#include <iostream>
#include <ros/package.h>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include "ymbot_amp_devel/amp_controller.hpp"
#include "ymbot_amp_devel/average_filter.hpp"
#include "ymbot_amp_devel/phase_generator.hpp"

// --- 新增头文件 ---
#include <geometry_msgs/Vector3Stamped.h>
#include <mutex>

using namespace std;
using namespace std::chrono;
#define ARM_MUB_LIB 10

typedef struct
{
    float angle_float[3];
    float gyro_float[3];
    float accel_float[3];
    float mag_float[3];
    float quat_float[4];
} YEIMUData;

struct realcfg
{
    std::string mujoco_model_path = "";
    double sim_duration = 120.0;
    double dt = 0.002;
    int decimation = 1;
    realcfg(const std::string &model_path) : mujoco_model_path(model_path) {}
};

// 全局标志位
bool loadControllerFlag_ = false;
bool setWalkFlag_ = false;
bool emergencyStopFlag_ = false;
bool standingModeFlag = false;
int errorState{0};
geometry_msgs::Twist cmd_vel;

int count_lowlevel = 0;
YEIMUData *data_imu = nullptr;
SharedMemory shm_motor_down(false);
SharedMemoryArm shm_arm_(false);
RecEncosMotorData recM[JOINT_MOTOR_NUMBER];
RecEncosMotorData recJ[JOINT_MOTOR_NUMBER];
SendEncosMotorData sendDataJoint[JOINT_MOTOR_NUMBER];
EuArmData recJ_arm[JOINT_ARM_NUMBER];
double pos_des_arm_[JOINT_ARM_NUMBER];
SharedDataArm sendDataJoint_arm[JOINT_ARM_NUMBER];

// 腿部读取数据向量
Eigen::VectorXd q = Eigen::VectorXd::Zero(JOINT_MOTOR_NUMBER - 1);
Eigen::VectorXd dq = Eigen::VectorXd::Zero(JOINT_MOTOR_NUMBER - 1);
// 手臂读取数据向量
Eigen::VectorXd q_arm = Eigen::VectorXd::Zero(ARM_MUB_LIB);
Eigen::VectorXd dq_arm = Eigen::VectorXd::Zero(ARM_MUB_LIB);
// 数据融合（13个腿部 + 10个手臂 = 23个）
Eigen::VectorXd q_combined = Eigen::VectorXd::Zero(JOINT_MOTOR_NUMBER - 1 + ARM_MUB_LIB);
Eigen::VectorXd dq_combined = Eigen::VectorXd::Zero(JOINT_MOTOR_NUMBER - 1 + ARM_MUB_LIB);

// 调试：打开文件用于写入数据
std::ofstream outfile("/home/pc/ymbot_e/src/data/sim2real_rec_joint.txt", std::ios::out);
std::ofstream outfile_send("/home/pc/ymbot_e/src/data/sim2real_send_joint.txt", std::ios::out);
std::ofstream onnx_rec("/home/pc/ymbot_e/src/data/sim2real_onnx_rec_joint.txt", std::ios::out);
std::ofstream onnx_send("/home/pc/ymbot_e/src/data/sim2real_onnx_send_joint.txt", std::ios::out);
std::ofstream motor_velocity("/home/pc/ymbot_e/src/data/sim2real_rec_velocity.txt", std::ios::out);

AverageFilter slid_filter(50);

// --------------------------------------------------------------------------------
// [新增] 全局变量与互斥锁，用于接收来自 reset_four_element 节点的数据
// --------------------------------------------------------------------------------
std::mutex sensor_mutex;
Eigen::VectorXd g_projected_gravity = Eigen::VectorXd::Zero(3);
Eigen::VectorXd g_base_lin_vel = Eigen::VectorXd::Zero(3);
Eigen::VectorXd g_base_ang_vel = Eigen::VectorXd::Zero(3);
double g_current_heading = 0.0; // [新增] 全局变量存储Heading

// [新增] 角度归一化函数
double wrap_to_pi(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

// [新增] 回调函数：Heading
void currentHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex);
    g_current_heading = msg->data;
}

// [新增] 回调函数：重力投影
void projectedGravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex);
    g_projected_gravity << msg->vector.x, msg->vector.y, msg->vector.z;
}

// [新增] 回调函数：线速度
void projectedVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex);
    g_base_lin_vel << msg->vector.x, msg->vector.y, msg->vector.z;
}

// [新增] 回调函数：角速度
void projectedOmegaCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex);
    g_base_ang_vel << msg->vector.x, msg->vector.y, msg->vector.z;
}
// --------------------------------------------------------------------------------

// [修改] 获取观测数据函数
// 返回值顺序: (关节位置, 关节速度, 四元数, 线速度, 角速度, 重力投影)
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> get_observation()
{
    // 1. 读取下半身电机数据
    shm_motor_down.readJointDatafromMotor(recJ);
    for (int i = 0; i < JOINT_MOTOR_NUMBER - 1; ++i)
    {
        q[i] = recJ[i].pos_;
        dq[i] = recJ[i].vel_;
    }

    // 2. 读取上半身电机数据
    shm_arm_.readJointDatafromMotorArm(recJ_arm);
    for (int i = 0; i < JOINT_ARM_NUMBER; ++i)
    {
        q_arm[i] = recJ_arm[i].pos_;
        dq_arm[i] = recJ_arm[i].vel_;
    }

    // 合并关节数据（仅用于记录，不传给RL）
    q_combined << q, q_arm;
    dq_combined << dq, dq_arm;

    // 3. 获取IMU四元数 (保留原始数据备用)
    Eigen::VectorXd quat(4);
    if (data_imu) {
        quat << data_imu->quat_float[0], data_imu->quat_float[1], data_imu->quat_float[2], data_imu->quat_float[3];
    } else {
        quat.setZero();
    }

    // 4. [修改核心] 从 ROS 订阅的全局变量中读取处理好的数据
    Eigen::VectorXd current_lin_vel(3);
    Eigen::VectorXd current_ang_vel(3);
    Eigen::VectorXd current_proj_grav(3);

    {
        std::lock_guard<std::mutex> lock(sensor_mutex);
        current_lin_vel = g_base_lin_vel;     // 来自 /projected_velocity
        current_ang_vel = g_base_ang_vel;     // 来自 /projected_omega
        current_proj_grav = g_projected_gravity; // 来自 /projected_gravity
    }

    // 只返回下肢13个关节的数据给RL，上肢单独控制
    return std::make_tuple(q, dq, quat, current_lin_vel, current_ang_vel, current_proj_grav);
}

void run_real(const realcfg &real_cfg, AMPController &amp_controller)
{
    int count_onnx = 0;
    int count_pd = 0;
    int count_pd_time = 0;
    static bool first_pos = true;
    Eigen::VectorXd first_q = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd pd_target_q(amp_controller.cfg.env.num_joints);

    // pd_target_q << -0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    //     -0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    //     0.0;
    pd_target_q <<  // -0.0125, -0.0833, -0.4380, -0.2151, 0.1007, 0.2604,
                     // -0.0065, 0.0617, 0.5289, 0.2141, 0.0960, -0.2703,
                    //  -0.0125, -0.0833, -0.4380, -0.2151, -0.1, 0.0,
                    // -0.0065, 0.0617, 0.5289, 0.2141, -0.1, 0.0,
                    -0.0065, 0.0833, 0.5, 0.2141, -0.1, 0.0,
                    -0.0065, -0.0833, -0.5, 0.2141, -0.1, 0.0,
        0.0;

    // 上肢固定位置（10个关节）
    double arm_fixed_pos[JOINT_ARM_NUMBER];
    for (int i = 0; i < JOINT_ARM_NUMBER; i++)
    {
        if (i < 5)  // 左臂5个关节
        {
            arm_fixed_pos[i] = 0.35;  // shoulder_pitch
            if (i == 1) arm_fixed_pos[i] = 0.0;   // shoulder_roll
            if (i == 2) arm_fixed_pos[i] = 0.0;    // shoulder_yaw
            if (i == 3) arm_fixed_pos[i] = -0.70;  // elbow
            if (i == 4) arm_fixed_pos[i] = 0.0;    // wrist
        }
        else if (i < 10)  // 右臂5个关节
        {
            arm_fixed_pos[i] = -0.35;  // shoulder_pitch
            if (i == 6) arm_fixed_pos[i] = 0.0;  // shoulder_roll
            if (i == 7) arm_fixed_pos[i] = 0.0;    // shoulder_yaw
            if (i == 8) arm_fixed_pos[i] = 0.70;  // elbow
            if (i == 9) arm_fixed_pos[i] = 0.0;    // wrist
        }
        else  // 其他关节
        {
            arm_fixed_pos[i] = 0.0;
        }
    }

    Eigen::VectorXd q_lab = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd dq_lab = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);

    Eigen::VectorXd target_q = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_dq = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_q_mjc(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_q_pd(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_dq_mjc(amp_controller.cfg.env.num_joints);

    // 定义 ONNX 需要的变量
    Eigen::VectorXd action = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints); 
    Eigen::VectorXd phase = Eigen::VectorXd::Zero(6); 
    
    // 创建 PhaseGenerator 对象，与 flyrobot-wjc-mujoco-sim 保持一致
    PhaseGenerator phase_generator; 

    auto cycle_duration = milliseconds(static_cast<int>(real_cfg.dt * 1000));
    auto next_time_point_motor = std::chrono::steady_clock::now() + cycle_duration;
    auto loop_start = std::chrono::high_resolution_clock::now();

    while (ros::ok())
    {
        // [新增] 必须调用 spinOnce 以确保回调函数能更新数据
        ros::spinOnce(); 

        loop_start = std::chrono::high_resolution_clock::now();

        try
        {
            // 构造函数体
        }
        catch (const std::length_error &e)
        {
            ROS_ERROR("std::length_error: %s", e.what());
            throw;
        }

        if (errorState == 0)
        {
            // [修改] 获取机器人状态，解包新的返回值
            // 注意：第4个参数是 base_linear_velocity，第5个是 base_angular_velocity
            auto [q_Motor, dq_Motor, quat, base_linear_velocity, base_angular_velocity, projected_gravity] = get_observation();

            if (motor_velocity.is_open())
            {
                motor_velocity << count_onnx * real_cfg.dt;
                for (int i = 0; i < projected_gravity.size(); ++i)
                {
                    motor_velocity << "," << projected_gravity[i];
                }
                motor_velocity << "\n";
            }

            Eigen::VectorXd pd_target_test(amp_controller.cfg.env.num_joints);
            pd_target_test << -0.1, -0.1, 0.0, 0.3, 0.3, 0.0,
                0.1, 0.1, 0.3, 0.3,
                0.20, 0.20, 0.0;
            
            if (first_pos)
            {
                first_q = q_Motor;
                ROS_INFO("=== Joint Mapping Information ===");
                for (int i = 0; i < amp_controller.cfg.env.num_joints; i++)
                {
                    size_t lab_idx = amp_controller.cfg.mjc2lab[i];
                    pd_target_q[i] *= amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                    ROS_INFO("MuJoCo[%d] -> IsaacLab[%zu]: %s, direction=%d, pd_target=%.3f", 
                             i, lab_idx, 
                             amp_controller.cfg.joint_params_isaaclab[lab_idx].joint_name.c_str(),
                             amp_controller.cfg.joint_params_isaaclab[lab_idx].direction,
                             pd_target_q[i]);
                }
                ROS_INFO("===============================");
                first_pos = false;
            }

            // 重排序并调整方向
            for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx)
            {
                size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
                q_lab[lab_idx] = q_Motor[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                dq_lab[lab_idx] = dq_Motor[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
            }

            if (loadControllerFlag_ == 1 && !emergencyStopFlag_)
            { 
                // 控制器模式
                if (setWalkFlag_ && !standingModeFlag)
                { 
                    // ONNX行走模式
                    ROS_INFO_THROTTLE(1.0, "Mode: ONNX Walking Mode");
                    if (onnx_rec.is_open())
                    {
                        onnx_rec << count_onnx * real_cfg.dt;
                        for (int i = 0; i < q_Motor.size(); ++i)
                        {
                            onnx_rec << "," << q_Motor[i];
                        }
                        onnx_rec << "\n";
                    }

                    if (count_onnx % real_cfg.decimation == 0)
                    {
                        Eigen::VectorXd velocity_commands(3);
                            velocity_commands << 
                            cmd_vel.linear.x,            // 前进方向补偿
                            cmd_vel.linear.y ,     // 侧向补偿
                            cmd_vel.angular.z ;     // Yaw补偿
                            
                        // [修改] 始终计算 Heading 偏置并叠加到指令中
                        // 使用从 topic 订阅得到的全局 heading
                        double current_heading;
                        {
                            std::lock_guard<std::mutex> lock(sensor_mutex);
                            current_heading = g_current_heading;
                        }
                        
                        // heading_target 为 0
                        double heading_target = 0.0;
                        double heading_error = wrap_to_pi(heading_target - current_heading);
                        
                        // stiffness = 0.5
                        double heading_control_stiffness = 0.5;
                        
                        // 计算 Heading 修正量 (作为已有的 yaw 指令的偏置)
                        double heading_correction_z = heading_control_stiffness * heading_error;
                        
                        // 叠加偏置
                        velocity_commands[2] += heading_correction_z;

                        // 限制范围 (假设范围为 [-1.0, 1.0])
                        double min_ang_vel = -1.0;
                        double max_ang_vel = 1.0;
                        velocity_commands[2] = std::clamp(velocity_commands[2], min_ang_vel, max_ang_vel);
                        
                        // ROS_INFO_THROTTLE(1.0, "Auto Heading: error=%.3f, cmd=%.3f", heading_error, velocity_commands[2]);
                        }

                        // [新增] 使用 PhaseGenerator 生成 phase 观测
                        // 注意: 因为我们在 decimation 之后调用，所以 dt 应该是 real_cfg.dt * decimation
                        // 否则相位更新速度会比实际慢 decimation 倍
                        double effective_dt = real_cfg.dt * real_cfg.decimation;
                        Eigen::Vector3d cmd_vel_vector(velocity_commands[0], velocity_commands[1], velocity_commands[2]);
                        phase = phase_generator.generatePhase(base_linear_velocity, cmd_vel_vector, effective_dt);

                        // [修改] 调用 ONNX 推理，使用从 ROS 订阅到的数据
                        amp_controller.onnx_output(base_linear_velocity, base_angular_velocity, projected_gravity, velocity_commands, q_lab, dq_lab, action, phase);

                        // [新增] 打印进入RL后的观测值
                        ROS_INFO("=== RL Observation Data ===");
                        ROS_INFO("base_linear_velocity: [%.4f, %.4f, %.4f]", 
                                 base_linear_velocity[0], base_linear_velocity[1], base_linear_velocity[2]);
                        ROS_INFO("base_angular_velocity: [%.4f, %.4f, %.4f]", 
                                 base_angular_velocity[0], base_angular_velocity[1], base_angular_velocity[2]);
                        ROS_INFO("projected_gravity: [%.4f, %.4f, %.4f]", 
                                 projected_gravity[0], projected_gravity[1], projected_gravity[2]);
                        ROS_INFO("velocity_commands: [%.4f, %.4f, %.4f]", 
                                 velocity_commands[0], velocity_commands[1], velocity_commands[2]);
                        
                        // 打印关节位置 q_lab
                        std::stringstream ss_q;
                        ss_q << "q_lab: [";
                        for (int i = 0; i < q_lab.size(); ++i) {
                            ss_q << std::fixed << std::setprecision(4) << q_lab[i];
                            if (i < q_lab.size() - 1) ss_q << ", ";
                        }
                        ss_q << "]";
                        ROS_INFO("%s", ss_q.str().c_str());
                        
                        // 打印关节速度 dq_lab
                        std::stringstream ss_dq;
                        ss_dq << "dq_lab: [";
                        for (int i = 0; i < dq_lab.size(); ++i) {
                            ss_dq << std::fixed << std::setprecision(4) << dq_lab[i];
                            if (i < dq_lab.size() - 1) ss_dq << ", ";
                        }
                        ss_dq << "]";
                        ROS_INFO("%s", ss_dq.str().c_str());
                        
                        // 打印动作 action
                        std::stringstream ss_action;
                        ss_action << "action: [";
                        for (int i = 0; i < action.size(); ++i) {
                            ss_action << std::fixed << std::setprecision(4) << action[i];
                            if (i < action.size() - 1) ss_action << ", ";
                        }
                        ss_action << "]";
                        ROS_INFO("%s", ss_action.str().c_str());
                        
                        // 打印相位 phase
                        std::stringstream ss_phase;
                        ss_phase << "phase: [";
                        for (int i = 0; i < phase.size(); ++i) {
                            ss_phase << std::fixed << std::setprecision(4) << phase[i];
                            if (i < phase.size() - 1) ss_phase << ", ";
                        }
                        ss_phase << "]";
                        ROS_INFO("%s", ss_phase.str().c_str());
                        ROS_INFO("===========================");

                        //  放缩和补偿
                        for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                        {
                            if (i >= amp_controller.cfg.joint_params_isaaclab.size()) {
                                std::cerr << "Error: Index " << i << " out of bounds for joint_params_isaaclab (size: " 
                                          << amp_controller.cfg.joint_params_isaaclab.size() << ")" << std::endl;
                                continue;
                            }
                            target_q[i] = amp_controller.actions_[i] * amp_controller.cfg.control.actions_;
                            target_q[i] += amp_controller.cfg.joint_params_isaaclab[i].offset;
                            target_q[i] = std::clamp(target_q[i], -amp_controller.cfg.control.clip_actions,
                                                     amp_controller.cfg.control.clip_actions);
                        }

                        // 重新排序lab到电机中的关节顺序和方向
                        ROS_INFO("=== RL Mode: Applying Direction Transform ===");
                        for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx)
                        {
                            size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
                            int direction = amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                            double target_q_before = target_q[lab_idx];
                            
                            target_q_mjc[mjc_idx] = target_q[lab_idx] * direction;
                            target_dq_mjc[mjc_idx] = target_dq[lab_idx] * direction;
                            
                            ROS_INFO("Joint[%zu->%zu] %s: target_q[lab]=%.4f, direction=%d, target_q_mjc=%.4f", 
                                     mjc_idx, lab_idx, 
                                     amp_controller.cfg.joint_params_isaaclab[lab_idx].joint_name.c_str(),
                                     target_q_before, direction, target_q_mjc[mjc_idx]);
                        }
                        ROS_INFO("============================================");

                        // 上半身控制：使用固定位置
                        for (int i = 0; i < JOINT_ARM_NUMBER; i++)
                        {
                            pos_des_arm_[i] = arm_fixed_pos[i];
                        }
                        shm_arm_.writeJointDatatoMotorArm(pos_des_arm_);

                        if (onnx_send.is_open())
                        {
                            onnx_send << count_onnx * real_cfg.dt;
                            for (int i = 0; i < target_q_pd.size(); ++i)
                            {
                                onnx_send << "," << target_q_pd[i];
                            }
                            onnx_send << "\n";
                        }
                    }

                    // 下半身：发送目标位置到电机
                    for (int i = 0; i < JOINT_MOTOR_NUMBER; ++i)
                    {
                        if (i < 13)
                        {
                            size_t lab_idx = amp_controller.cfg.mjc2lab[i];
                            sendDataJoint[i].pos_des_ = target_q_mjc[i];
                            sendDataJoint[i].vel_des_ = target_dq_mjc[i];
                            sendDataJoint[i].kp_ = amp_controller.cfg.joint_params_isaaclab[lab_idx].kp * 0.5;
                            sendDataJoint[i].kd_ = amp_controller.cfg.joint_params_isaaclab[lab_idx].kd;
                            sendDataJoint[i].ff_ = 0.0;
                        }
                        if (i == 13)
                        {
                            sendDataJoint[i].pos_des_ = 0.0;
                            sendDataJoint[i].vel_des_ = 0.0;
                            sendDataJoint[i].kp_ = 250;
                            sendDataJoint[i].kd_ = 5;
                            sendDataJoint[i].ff_ = 0.0;
                        } 
                    }
                    shm_motor_down.writeJointDatatoMotor(sendDataJoint);
                }
                else
                {
                    // PD站立模式
                    ROS_INFO_THROTTLE(1.0, "Mode: PD Standing Mode - Lower body PD control active");
                    const double pd_total_time = 1.0;
                    count_pd_time = std::min(count_pd_time, 65536);
                    static int pd_count_time = (1 / real_cfg.dt) - 1;
                    count_pd = std::min(count_pd, pd_count_time);
                    
                    // 修正：为JOINT_MOTOR_NUMBER个关节分配空间，而不是只为13个
                    Eigen::VectorXd kp(JOINT_MOTOR_NUMBER);
                    kp.setConstant(200); 

                    Eigen::VectorXd kd(JOINT_MOTOR_NUMBER);
                    kd.setConstant(5);  // 增加阻尼以提高稳定性

                    for (int i = 0; i < amp_controller.cfg.env.num_joints; i++)
                    {
                        target_q_pd[i] = first_q[i] + ((count_pd * real_cfg.dt) / pd_total_time) * (pd_target_q[i] - first_q[i]);
                    }

                    if (outfile.is_open())
                    {
                        outfile << count_pd_time * real_cfg.dt;
                        for (int i = 0; i < q_Motor.size(); ++i)
                        {
                            outfile << "," << q_Motor[i];
                        }
                        outfile << "\n";
                    }
                    if (outfile_send.is_open())
                    {
                        outfile_send << count_pd_time * real_cfg.dt;
                        for (int i = 0; i < target_q_pd.size(); ++i)
                        {
                            outfile_send << "," << target_q_pd[i];
                        }
                        outfile_send << "\n";
                    }

                    for (int i = 0; i < JOINT_MOTOR_NUMBER; ++i)
                    {
                        if (i < 13)
                        {
                            sendDataJoint[i].pos_des_ = target_q_pd[i];
                            sendDataJoint[i].vel_des_ = 0.0;
                            sendDataJoint[i].kp_ = kp[i];
                            sendDataJoint[i].kd_ = kd[i];
                            sendDataJoint[i].ff_ = 0.0;
                            
                            // 添加调试信息（每秒输出一次）
                            if (count_pd_time % 100 == 0) {
                                ROS_INFO("Joint[%d] (Leg): pos_des=%.3f, kp=%.1f, kd=%.1f", 
                                         i, sendDataJoint[i].pos_des_, sendDataJoint[i].kp_, sendDataJoint[i].kd_);
                            }
                        }
                        else // i == 13, 腰部关节
                        {
                            sendDataJoint[i].pos_des_ = 0.0;
                            sendDataJoint[i].vel_des_ = 0.0;
                            sendDataJoint[i].kp_ = kp[i];
                            sendDataJoint[i].kd_ = kd[i];
                            sendDataJoint[i].ff_ = 0.0;
                            
                            // 为腰部关节添加单独的调试信息
                            if (count_pd_time % 100 == 0) {
                                ROS_INFO("Joint[%d] (Waist): pos_des=%.3f, kp=%.1f, kd=%.1f", 
                                         i, sendDataJoint[i].pos_des_, sendDataJoint[i].kp_, sendDataJoint[i].kd_);
                            }
                        }
                    }
                    shm_motor_down.writeJointDatatoMotor(sendDataJoint);
                    
                    // 添加PD控制状态总结（每秒输出一次），确认所有14个关节都在PD控制下
                    if (count_pd_time % 100 == 0) {
                        ROS_INFO("=== PD Control Summary ===");
                        ROS_INFO("All 14 joints are under PD control.");
                        ROS_INFO("Joints 0-12 (Legs): kp=%.1f, kd=%.1f", kp[0], kd[0]);
                        ROS_INFO("Joint 13 (Waist): kp=%.1f, kd=%.1f", kp[13], kd[13]);
                        ROS_INFO("Transition progress: %.2f / %.2f s", count_pd * real_cfg.dt, pd_total_time);
                        ROS_INFO("========================");
                    }
                    
                    // 上半身控制：使用固定位置
                    for (int i = 0; i < JOINT_ARM_NUMBER; i++)
                    {
                        pos_des_arm_[i] = arm_fixed_pos[i];
                    }
                    shm_arm_.writeJointDatatoMotorArm(pos_des_arm_);
                    count_pd_time++;
                    count_pd++;
                }
            }
            else
            {
                // 未加载控制器或紧急停止
                ROS_INFO_THROTTLE(1.0, "Mode: Controller not loaded or emergency stop - loadControllerFlag_=%s, emergencyStopFlag_=%s", 
                                  loadControllerFlag_ ? "true" : "false", 
                                  emergencyStopFlag_ ? "true" : "false");
                for (int i = 0; i < JOINT_MOTOR_NUMBER; ++i)
                {
                    sendDataJoint[i].pos_des_ = 0.0;
                    sendDataJoint[i].vel_des_ = 0;
                    sendDataJoint[i].kp_ = 0;
                    sendDataJoint[i].kd_ = 0;
                    sendDataJoint[i].ff_ = 0;
                }
                shm_motor_down.writeJointDatatoMotor(sendDataJoint);
            }
        }

        count_onnx++;
        if (count_onnx == 65536)
        {
            count_onnx = 1;
        }
        std::this_thread::sleep_until(next_time_point_motor);
        next_time_point_motor += cycle_duration;
    }
}

void EmergencyStopCallback(const std_msgs::Float32::ConstPtr &msg)
{
    emergencyStopFlag_ = true;
    ROS_INFO("Emergency Stop !!!!!!");
    cout << "EmergencyStopCallback" << endl;
}

void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel = *msg;
}

void setWalkCallback(const std_msgs::Float32::ConstPtr &msg)
{
    if (loadControllerFlag_)
    {
        setWalkFlag_ = true;
        standingModeFlag = false;
        ROS_INFO("Set WALK Mode");
    }
}

void loadControllerCallback(const std_msgs::Float32::ConstPtr &msg)
{
    loadControllerFlag_ = true;
    standingModeFlag = true;
    ROS_INFO(" loadControllerFlag_");
}

void Walk2stanceCallback(const std_msgs::Float32::ConstPtr &msg)
{
    if (loadControllerFlag_ && setWalkFlag_)
    {
        ROS_INFO(" Walk2stance");
        setWalkFlag_ = false;
        standingModeFlag = true;
    }
}

std::vector<std::string> parseStringArrayParam(const std::string &param)
{
    std::vector<std::string> result;
    std::stringstream ss(param.substr(1, param.size() - 2)); 
    std::string item;
    while (std::getline(ss, item, ','))
    {
        item.erase(std::remove(item.begin(), item.end(), '\"'), item.end());
        item.erase(std::remove(item.begin(), item.end(), '\''), item.end());
        item.erase(0, item.find_first_not_of(" \t\n\r\f\v"));
        item.erase(item.find_last_not_of(" \t\n\r\f\v") + 1);
        if (!item.empty()) {
            result.push_back(item);
        }
    }
    return result;
}

std::vector<double> parseParam(const std::string &param)
{
    std::vector<double> result;
    // 检查参数是否以 '[' 开头和以 ']' 结尾
    if (param.length() < 2 || param.front() != '[' || param.back() != ']') {
        ROS_ERROR("Invalid parameter format: %s", param.c_str());
        return result;
    }
    
    std::stringstream ss(param.substr(1, param.size() - 2)); // 去掉方括号
    std::string item;
    while (std::getline(ss, item, ','))
    {
        // 去除前后空格
        item.erase(0, item.find_first_not_of(" \t\n\r\f\v"));
        item.erase(item.find_last_not_of(" \t\n\r\f\v") + 1);
        
        if (!item.empty()) {
            try {
                double value = std::stod(item);
                result.push_back(value);
            } catch (const std::invalid_argument& ia) {
                ROS_ERROR("Invalid argument in parameter: %s, value: %s, reason: %s", param.c_str(), item.c_str(), ia.what());
                continue;
            } catch (const std::out_of_range& oor) {
                ROS_ERROR("Out of range in parameter: %s, value: %s, reason: %s", param.c_str(), item.c_str(), oor.what());
                continue;
            }
        }
    }
    return result;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rl_sim2sim");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // [修改] 增加线程数以确保回调及时响应

    spinner.start();
    
    //------------------------imu-------------------------------------------------
    const char *SHM_NAME = "/imu_shared";
    const size_t SHM_SIZE = sizeof(YEIMUData);

    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1)
    {
        perror("shm_open");
        // 如果这里返回，可能导致后面运行不了，根据需要决定是否return
        // return 0; 
    } else {
        data_imu = (YEIMUData *)mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
        if (data_imu == MAP_FAILED)
        {
            perror("mmap");
            return 0;
        }
    }
    
    //---------------------------leg & arm----------------------------------
    std::cerr << "Initializing motors..." << std::endl;
    for (int i = 0; i < JOINT_ARM_NUMBER; i++) 
    {
        if (i < 4) pos_des_arm_[i] = 0.0;
        else if (4 < i && i < 9) pos_des_arm_[i] = 0.0;
        else if (i == 4 || i == 9 || i == 10 || i == 11) pos_des_arm_[i] = 0.0;
    }
    shm_arm_.readJointDatafromMotorArm(recJ_arm);
    shm_arm_.writeJointDatatoMotorArm(pos_des_arm_);

    std::string model_path, onnx_path, joint_names_str, kps_str, kds_str, offsets_str, tau_limits_str, directions_str,
        mujoco_names_str;

    int num_joints;

    if (!nh.getParam("model_path", model_path)) return 1;
    if (!nh.getParam("num_joints", num_joints)) return 1;
    if (!nh.getParam("onnx_path", onnx_path)) return 1;

    if (!nh.getParam("joint_names", joint_names_str) || !nh.getParam("kps", kps_str) || !nh.getParam("kds", kds_str) ||
        !nh.getParam("offsets", offsets_str) || !nh.getParam("tau_limits", tau_limits_str) ||
        !nh.getParam("directions", directions_str) || !nh.getParam("joint_names_mujoco", mujoco_names_str))
    {
        ROS_ERROR("Failed to get required parameters from parameter server");
        return -1;
    }

    std::vector<std::string> joint_names = parseStringArrayParam(joint_names_str);
    std::vector<double> kps = parseParam(kps_str);
    std::vector<double> kds = parseParam(kds_str);
    std::vector<double> offsets = parseParam(offsets_str);
    std::vector<double> tau_limits = parseParam(tau_limits_str);
    std::vector<double> directions = parseParam(directions_str);
    std::vector<std::string> joint_names_mujoco = parseStringArrayParam(mujoco_names_str);

    // 调试：打印解析的参数数量
    ROS_INFO("Parsed parameters:");
    ROS_INFO("  joint_names: %zu", joint_names.size());
    ROS_INFO("  kps: %zu", kps.size());
    ROS_INFO("  kds: %zu", kds.size());
    ROS_INFO("  offsets: %zu", offsets.size());
    ROS_INFO("  tau_limits: %zu", tau_limits.size());
    ROS_INFO("  directions: %zu", directions.size());
    ROS_INFO("  joint_names_mujoco: %zu", joint_names_mujoco.size());
    ROS_INFO("  num_joints: %d", num_joints);

    // [新增] 订阅 reset_four_element 发布的处理后数据
    ros::Subscriber sub_grav = nh.subscribe<geometry_msgs::Vector3Stamped>("/projected_gravity", 1, projectedGravityCallback);
    ros::Subscriber sub_vel = nh.subscribe<geometry_msgs::Vector3Stamped>("/projected_velocity", 1, projectedVelocityCallback);
    ros::Subscriber sub_omega = nh.subscribe<geometry_msgs::Vector3Stamped>("/projected_omega", 1, projectedOmegaCallback);
    ros::Subscriber sub_heading = nh.subscribe<std_msgs::Float32>("/current_heading", 1, currentHeadingCallback); // [新增] Heading 订阅

    ros::Subscriber subSetWalk_ = nh.subscribe<std_msgs::Float32>("/set_walk", 1, setWalkCallback);
    ros::Subscriber subLoadcontroller_ = nh.subscribe<std_msgs::Float32>("/load_controller", 1, loadControllerCallback);
    ros::Subscriber subEmgstop_ = nh.subscribe<std_msgs::Float32>("/emergency_stop", 1, EmergencyStopCallback);
    ros::Subscriber cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, CmdvelCallback);
    ros::Subscriber subWalk2stance_ = nh.subscribe<std_msgs::Float32>("/set_walk2stance", 1, Walk2stanceCallback);

    std::vector<AMPController::Cfg::JointParams> joint_params_isaaclab;
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        AMPController::Cfg::JointParams params;
        params.joint_name = joint_names[i];
        params.kp = kps[i];
        params.kd = kds[i];
        params.offset = offsets[i];
        params.tau_limit = tau_limits[i];
        params.direction = static_cast<int>(directions[i]);
        joint_params_isaaclab.push_back(params);
    }

    realcfg real_cfg(model_path);
    ROS_INFO("Creating AMPController with num_joints=%d", num_joints);
    AMPController amp_controller(onnx_path, num_joints, joint_params_isaaclab, joint_names_mujoco);
    ROS_INFO("AMPController created successfully");
    
    // [新增] 读取 action_clip_min 和 action_clip_max 参数
    std::vector<double> action_clip_min_vec = getParamVec(nh, "action_clip_min");
    std::vector<double> action_clip_max_vec = getParamVec(nh, "action_clip_max");
    
    // 如果参数不存在，提供默认值 (可选，或者在控制器中处理空值)
    if (action_clip_min_vec.empty() || action_clip_max_vec.empty()) {
        ROS_WARN("Action clip parameters not found or empty. Using default clipping.");
    } else {
        ROS_INFO("Action clip parameters loaded. Size: min=%lu, max=%lu", action_clip_min_vec.size(), action_clip_max_vec.size());
    }
    
    // 3. 运行控制循环
    amp_controller.cfg.control.action_clip_min = action_clip_min_vec;
    amp_controller.cfg.control.action_clip_max = action_clip_max_vec;

    run_real(real_cfg, amp_controller);
    
    ros::waitForShutdown();

    return 0;
}