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
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <xmlrpcpp/XmlRpcValue.h>             // 添加这行
#include "ymbot_amp_devel/amp_controller.hpp" // 控制器
#include "ymbot_amp_devel/average_filter.hpp"

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
    double dt = 0.01;
    int decimation = 1;
    realcfg(const std::string &model_path) : mujoco_model_path(model_path) {}
};
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
// 数据融合
Eigen::VectorXd q_combined(q.size() + q_arm.size());
Eigen::VectorXd dq_combined(dq.size() + dq_arm.size());
// 调试：打开文件用于写入数据
std::ofstream outfile("/home/pc/ymbot_e/src/data/sim2real_rec_joint.txt", std::ios::out);
std::ofstream outfile_send("/home/pc/ymbot_e/src/data/sim2real_send_joint.txt", std::ios::out);
std::ofstream onnx_rec("/home/pc/ymbot_e/src/data/sim2real_onnx_rec_joint.txt", std::ios::out);
std::ofstream onnx_send("/home/pc/ymbot_e/src/data/sim2real_onnx_send_joint.txt", std::ios::out);
std::ofstream motor_velocity("/home/pc/ymbot_e/src/data/sim2real_rec_velocity.txt", std::ios::out);

AverageFilter slid_filter(50);

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> get_observation() //, SharedMemory &shm_motor_leg, SharedMemoryArm &shm_arm
{
    shm_motor_down.readJointDatafromMotor(recJ);
    for (int i = 0; i < JOINT_MOTOR_NUMBER - 1; ++i)
    {
        q[i] = recJ[i].pos_;
        dq[i] = recJ[i].vel_;
    }

    shm_arm_.readJointDatafromMotorArm(recJ_arm);
    for (int i = 0; i < JOINT_ARM_NUMBER ; ++i)
    {

        q_arm[i] = recJ_arm[i].pos_;
        dq_arm[i] = recJ_arm[i].vel_;

    }

    // 合并下半身和上半身的关节数据
    q_combined << q, q_arm;
    dq_combined << dq, dq_arm;

    // for (int i = 0; i < 23; i++)
    // {
    //     std::cerr << "q_combined : [" << i << "]    " << q_combined[i] << std::endl;
    // }
    // 低通滤波处理dq
    // static Eigen::VectorXd filtered_dq = dq_combined;
    // filtered_dq = 0.2 * dq_combined + 0.8 * filtered_dq;
    // dq_combined = filtered_dq;
    // // 或者使用移动平均滤波(二选一)
    // dq_history.push_back(dq);
    // if (dq_history.size() > filter_window) {
    //     dq_history.pop_front();
    // }
    // Eigen::VectorXd filtered_dq = Eigen::VectorXd::Zero(dq.size());
    // for (const auto& d : dq_history) {
    //     filtered_dq += d;
    // }
    // filtered_dq /= dq_history.size();

    // 获取四元数，mujoco中默认四元数顺序为 [w, x, y, z] 格式
    Eigen::VectorXd quat(4);
    Eigen::VectorXd lin_acc = Eigen::VectorXd::Zero(3); // 线加速度
    Eigen::VectorXd ang_vel = Eigen::VectorXd::Zero(3); // 角速度

    quat << data_imu->quat_float[0], data_imu->quat_float[1], data_imu->quat_float[2], data_imu->quat_float[3];
    lin_acc << data_imu->accel_float[0], data_imu->accel_float[1], data_imu->accel_float[2];
    ang_vel << data_imu->angle_float[0], data_imu->angle_float[1], data_imu->angle_float[2];
    // 计算重力向量在机器人基坐标系下的投影
    Eigen::VectorXd projected_gravity = Eigen::VectorXd::Zero(3);

    if (quat.norm() > 0)
    {
        // 将四元数转换为旋转矩阵
        Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
        Eigen::Matrix3d rot_mat = q.toRotationMatrix();

        // 重力在世界坐标系中的方向(假设为Z轴负方向)
        Eigen::Vector3d world_gravity(0.0, 0.0, -1.0);

        // 将重力转换到机体坐标系
        projected_gravity = rot_mat.transpose() * world_gravity;
    }

    // 归一化重力向量(可选)
    if (projected_gravity.norm() > 0)
    {
        projected_gravity.normalize();
    }
    // projected_gravity = slid_filter.filter(projected_gravity);
    // std::cerr << "projected_gravity:  " << projected_gravity.transpose() << std::endl;
    return std::make_tuple(q_combined, dq_combined, quat, lin_acc, ang_vel, projected_gravity);
}

void run_real(const realcfg &real_cfg, AMPController &amp_controller)
{
    int count_onnx = 0;
    int count_pd = 0;
    int count_pd_time = 0;
    static bool first_pos = true;
    Eigen::VectorXd first_q = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd pd_target_q(amp_controller.cfg.env.num_joints);
    //  pd_target_q << -0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    //     0.0, 0.0,
    //     0.35, 0.18, 0.0, 0.87,
    //     0.35, -0.18, 0.0, 0.87;  
    pd_target_q << -0.15, 0.0, 0.0, 0.35, -0.22, 0.0,
                   -0.15, 0.0, 0.0, 0.35, -0.22, 0.0,
                    0.0,
                    0.35,  0.18, 0.0, -0.70, 0.0, 
                    0.35, -0.18, 0.0, -0.70, 0.0;
    // pd_target_q << 0.2, 0.2, 0.2, 0.2, 0.20, 0.2,
    //     0.2, 0.2, 0.2, 0.2, 0.20, 0.2,
    //     0.0, 0.0,
    //     0.35, 0.18, 0.0, 0.87,
    //     0.35, -0.18, 0.0, 0.87;

    // cmd
    Eigen::VectorXd velocity_commands = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd q_lab = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd dq_lab = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);

    Eigen::VectorXd target_q = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_dq = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_q_mjc(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_q_pd(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_dq_mjc(amp_controller.cfg.env.num_joints);

    auto cycle_duration = milliseconds(static_cast<int>(real_cfg.dt * 1000));
    auto next_time_point_motor = std::chrono::steady_clock::now() + cycle_duration;
    auto loop_start = std::chrono::high_resolution_clock::now();

    while (ros::ok())
    {
        // auto dt_cycle = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - loop_start).count();
        loop_start = std::chrono::high_resolution_clock::now();

        try
        {
            // 构造函数体
        }
        catch (const std::length_error &e)
        {
            // 处理异常
            ROS_ERROR("std::length_error: %s", e.what());
            throw; // 或者采取其他恢复措施
        }
        if (errorState == 0)
        {
            // 获取机器人状态
            auto [q_Motor, dq_Motor, quat, acc, angle_vel, projected_gravity] = get_observation(); //, shm_motor_down, shm_arm_
            if (motor_velocity.is_open())
            {
                motor_velocity << count_onnx * real_cfg.dt;
                for (int i = 0; i < projected_gravity.size(); ++i)
                {
                    motor_velocity << "," << projected_gravity[i];
                }
                motor_velocity << "\n";
            }
            // for (int i = 0; i < 8; i++)
            // {
            //     std::cerr << "dq_Motor_Left :" << i + 14 << "     " << dq_Motor[i + 14] << std::endl;
            // }
            Eigen::VectorXd pd_target_test(amp_controller.cfg.env.num_joints);
            pd_target_test << -0.1, -0.1, 0.0, 0.3, 0.3, 0.0,
                0.1, 0.1, 0.3, 0.3,
                0.20, 0.20, 0.18, -0.18,
                -0.2, -0.20, 0.1, 0.1,
                0.1, 0.1, -0.50, -0.50,0.0;
            if (first_pos)
            {
                first_q = q_Motor;

                for (int i = 0; i < amp_controller.cfg.env.num_joints; i++)
                {
                    size_t lab_idx = amp_controller.cfg.mjc2lab[i];
                    pd_target_q[i] *= amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                    std::cerr << "first_Position: " << i << "  lab: " << lab_idx << "  : " << amp_controller.cfg.joint_params_isaaclab[i].direction
                              << "  pd_pos: " << pd_target_q[i] << std::endl;
                }
                // for (int i = 0; i < 22; i++)
                // {
                //     std::cerr << "UP: pd_target_q :" << i << "    " << pd_target_q[i] << "      pd_target_q : " << pd_target_q[i] << std::endl;
                // }
                first_pos = false;
            }

            // 重排序并调整方向
            for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx)
            {
                size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
                // q_Motor[mjc_idx] *= amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                // dq_Motor[mjc_idx] *= amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                q_lab[lab_idx] = q_Motor[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                dq_lab[lab_idx] = dq_Motor[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                // pd_target_test[mjc_idx] *= amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                // q_lab[lab_idx] = pd_target_test[mjc_idx];
            }
            // for (int i = 0; i < 22; i++)
            // {
            //     std::cerr << "UP: q_lab :" << i << "    " << q_lab[i] << "      dq_lab : " << dq_lab[i] << std::endl;
            // }

            if (loadControllerFlag_ == 1 && !emergencyStopFlag_)
            { // 控制器
                if (setWalkFlag_ && !standingModeFlag)
                { // 控制器

                    // 保存lab顺序的关节接收顺序
                    if (onnx_rec.is_open())
                    {
                        onnx_rec << count_onnx * real_cfg.dt;
                        for (int i = 0; i < q_Motor.size(); ++i)
                        {
                            onnx_rec << "," << q_Motor[i];
                        }
                        onnx_rec << "\n";
                    }
                    // std::cerr << "main loop:" << count_onnx << endl;
                    if (count_onnx % real_cfg.decimation == 0)
                    {
                        // std::cerr << "onnx loop:" << count_onnx << endl;
                        Eigen::VectorXd velocity_commands(3);
                        velocity_commands << 
                            cmd_vel.linear.x ,    // 前进方向补偿（增强）+ 0.05
                            cmd_vel.linear.y + 0.05 ,//+ 0.05 ,      // 侧向补偿（向左） + 0.05    + 0.1
                            cmd_vel.angular.z + 0.1 ;//+ 0.1;    // Yaw补偿（减少右旋）+ 0.05          + 0.1
                            if(cmd_vel.angular.z !=0)
                            {
                                velocity_commands[1]-=0;
                                velocity_commands[2]+=0;


                            }
                        // gsj
                        amp_controller.onnx_output(projected_gravity, angle_vel, velocity_commands, q_lab, dq_lab);

                        //  放缩和补偿
                        for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                        {
                            target_q[i] = amp_controller.actions_[i] * amp_controller.cfg.control.action_scale;
                            target_q[i] += amp_controller.cfg.joint_params_isaaclab[i].offset;
                            target_q[i] = std::clamp(target_q[i], -amp_controller.cfg.control.clip_actions,
                                                     amp_controller.cfg.control.clip_actions);
                            // target_q[i] = pd_target_test[i];
                        }
                        // // 限幅和调整方向
                        // for (size_t i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                        // {
                        //     // tau_lab[i] = std::clamp(
                        //     //     tau_lab[i],
                        //     //     -amp_controller.cfg.joint_params_isaaclab[i].tau_limit,
                        //     //     amp_controller.cfg.joint_params_isaaclab[i].tau_limit);
                        //     // tau_lab[i] *= amp_controller.cfg.joint_params_isaaclab[i].direction;
                        //     target_q[i] *= amp_controller.cfg.joint_params_isaaclab[i].direction;
                        //     target_dq[i] *= amp_controller.cfg.joint_params_isaaclab[i].direction;
                        // }

                        // 重新排序lab到电机中的关节顺序和方向
                        for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx)
                        {
                            size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
                            // std::cerr << "mjc_sequence : " << mjc_idx << "    lib_sequence  :" << lab_idx << std::endl;
                            // std::cerr << "direction_sequence: " << amp_controller.cfg.joint_params_isaaclab[mjc_idx].direction << std::endl;
                            target_q_mjc[mjc_idx] = target_q[lab_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                            // std::cout << "target_q_mjc[" << mjc_idx << "] = " << target_q_mjc[mjc_idx] << std::endl;
                            // target_q_mjc[mjc_idx] = q_lab[lab_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                            target_dq_mjc[mjc_idx] = target_dq[lab_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                            // std::cerr << "target_q :" << mjc_idx << "    " << target_q_mjc[mjc_idx] << std::endl;
                        }
                        // for (int i = 0; i < 23; i++)
                        // {
                        //     std::cerr << "q_lab : [" << i << "]    " << target_q_mjc[i] << "      dq_lab : " << target_dq_mjc[i] << std::endl;
                        // }
                        // 上半身：通过ctl_arm函数发送目标位置

                       
                        
                        for (int i = 0; i < JOINT_ARM_NUMBER; i++) // 有四个电机不在控制器中但是需要发送零位置，4，9，10，11号电机
                        {
                            if (i < 10)
                            {
                                pos_des_arm_[i] = target_q_mjc[i + 13];
                            }
                            else if (i==11 || i ==10)
                            {
                                pos_des_arm_[i] = 0.0;
                            }
                            // else if (i == 11 || i == 10)
                            // {
                            //     double t = count_onnx * real_cfg.dt;
                            //     double amplitude = 0.1; // 小幅度摆动
                            //     double freq = 1.0;       // 1 Hz 摆动频率
                            //     pos_des_arm_[i] = amplitude * std::sin(2 * M_PI * freq * t);
                            // }
                        }
                        // for (int i = 0; i < JOINT_ARM_NUMBER; i++)
                        // {
                        //     std::cerr << "pos_des_arm_ : [" << i << "]    " << pos_des_arm_[i] << std::endl;
                        // }
                        shm_arm_.writeJointDatatoMotorArm(pos_des_arm_);
                        // auto loop_onnx = std::chrono::high_resolution_clock::now();
                        // auto dt_onnx = std::chrono::duration_cast<std::chrono::microseconds>(loop_onnx - loop_start).count();
                        // std::cout << " ms, onnx: " << dt_onnx / 1e3 << std::endl;
                        if (onnx_send.is_open())
                        {
                            onnx_send << count_onnx * real_cfg.dt;
                            for (int i = 0; i < target_q_mjc.size(); ++i)
                            {
                                onnx_send << "," << target_q_mjc[i];
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
                            sendDataJoint[i].pos_des_ = target_q_mjc[i]; //
                            sendDataJoint[i].vel_des_ = target_dq_mjc[i];
                            sendDataJoint[i].kp_ = amp_controller.cfg.joint_params_isaaclab[lab_idx].kp *1.0;
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

                    // for (int i = 0; i < JOINT_MOTOR_NUMBER; i++)
                    // {
                    //     std::cerr << "sendDataJoint : [" << i << "]    " << sendDataJoint[i].pos_des_ << std::endl;
                    // }
                    shm_motor_down.writeJointDatatoMotor(sendDataJoint);
                }
                else
                {
                    // PD站立
                    const double pd_total_time = 1.0;               // 总时间: 秒
                    count_pd_time = std::min(count_pd_time, 65536); // 记录数据时长
                    static int pd_count_time = (1 / real_cfg.dt) - 1;
                    count_pd = std::min(count_pd, pd_count_time); // 主函数周期
                    // 定义一组额外的kp 和 kd，用于站立
                    Eigen::VectorXd kp(amp_controller.cfg.env.num_joints);
                    kp.setConstant(200); // 一次性设置所有元素为200

                    Eigen::VectorXd kd(amp_controller.cfg.env.num_joints);
                    kd.setConstant(2); // 同样方法设置kds
                    for (int i = 0; i < amp_controller.cfg.env.num_joints; i++)
                    {
                        target_q_pd[i] = first_q[i] + ((count_pd * real_cfg.dt) / pd_total_time) * (pd_target_q[i] - first_q[i]);
                        // std::cerr << "target_Position: " << i << "  : " << target_q_pd[i] << std::endl;
                    }
                    // std::cerr << "q_Motor size: " << q_Motor.size() << std::endl;
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

                    // 下半身：发送目标位置到电机
                    for (int i = 0; i < JOINT_MOTOR_NUMBER; ++i)
                    {
                        if (i < 13)
                        {
                            sendDataJoint[i].pos_des_ = target_q_pd[i];
                            sendDataJoint[i].vel_des_ = 0.0;
                            sendDataJoint[i].kp_ = kp[i];
                            sendDataJoint[i].kd_ = kd[i];
                            sendDataJoint[i].ff_ = 0.0;
                        }
                        if (i == 13)
                        {
                            sendDataJoint[i].pos_des_ = 0.0;
                            sendDataJoint[i].vel_des_ = 0.0;
                            sendDataJoint[i].kp_ = kp[i];
                            sendDataJoint[i].kd_ = kd[i];
                            sendDataJoint[i].ff_ = 0.0;
                        }
                        // std::cout << "pd_pos[" << i << "] " << target_q_pd[i] << std::endl;
                    }
                    shm_motor_down.writeJointDatatoMotor(sendDataJoint);
                    // 上半身：通过ctl_arm函数发送目标位置
                    for (int i = 0; i < JOINT_ARM_NUMBER; i++) // 有四个电机不在控制器中但是需要发送零位置，4，9，10，11号电机
                    {
                        if (i < 10)
                        {
                            pos_des_arm_[i] = target_q_pd[i + 13];
                        }
                        else if (i == 10 || i == 11)
                        {
                            pos_des_arm_[i] = 0.0;
                        }
                    }
                    shm_arm_.writeJointDatatoMotorArm(pos_des_arm_);
                    // cout << "count_pd: " << count_pd << endl;
                    count_pd_time++;
                    count_pd++;
                    // if (count_pd < pd_total_time / real_cfg.dt)
                    // {
                    //
                    // }
                }
            }
            else
            {
                // 未加载控制器或紧急停止
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
        // auto loop_end = std::chrono::high_resolution_clock::now();
        // auto dt_total = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();
        // std::cout << " ms, total: " << dt_total / 1e3 << std::endl;
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
    // std::cout << " receive cmd vel ----   " << cmd_vel.linear.x << std::endl;
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
        // setStanceMode();
        standingModeFlag = true;
    }
}

// 修改 parseParam 函数，添加字符串数组解析支持
std::vector<std::string> parseStringArrayParam(const std::string &param)
{
    std::vector<std::string> result;
    std::stringstream ss(param.substr(1, param.size() - 2)); // 去掉方括号
    std::string item;
    while (std::getline(ss, item, ','))
    {
        // 去除引号和前后空格
        item.erase(std::remove(item.begin(), item.end(), '\"'), item.end());
        item.erase(std::remove(item.begin(), item.end(), '\''), item.end());
        item.erase(0, item.find_first_not_of(" \t\n\r\f\v"));
        item.erase(item.find_last_not_of(" \t\n\r\f\v") + 1);
        result.push_back(item);
    }
    return result;
}

// 用于从 launch 文件中解析参数
std::vector<double> parseParam(const std::string &param)
{
    std::vector<double> result;
    std::stringstream ss(param.substr(1, param.size() - 2)); // 去掉方括号
    std::string item;
    while (std::getline(ss, item, ','))
    {
        result.push_back(std::stod(item));
    }
    return result;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rl_sim2sim");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);

    spinner.start();
    //------------------------imu-------------------------------------------------
    const char *SHM_NAME = "/imu_shared";
    const size_t SHM_SIZE = sizeof(YEIMUData);

    int shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1)
    {
        perror("shm_open");
        // exit(1);
        return 0;
    }
    data_imu = (YEIMUData *)mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (data_imu == MAP_FAILED)
    {
        perror("mmap");
        // exit(1);
        return 0;
    }
    //---------------------------leg----------------------------------
    std::cerr << "1111111111111111111" << std::endl;
    // 打开电机共享内存
    //---------------------------arm----------------------------------
    std::cerr << "2222222222222222222" << std::endl;
    for (int i = 0; i < JOINT_ARM_NUMBER; i++) // 有四个电机不在控制器中但是需要发送零位置，4，9，10，11号电机
    {
        if (i < 4)
        {
            pos_des_arm_[i] = 0.0;
        }
        else if (4 < i && i < 9)
        {
            pos_des_arm_[i] = 0.0;
        }
        else if (i == 4 || i == 9 || i == 10 || i == 11)
        {
            pos_des_arm_[i] = 0.0;
        }
    }
    shm_arm_.readJointDatafromMotorArm(recJ_arm);
    shm_arm_.writeJointDatatoMotorArm(pos_des_arm_);

    std::string model_path, onnx_path, joint_names_str, kps_str, kds_str, offsets_str, tau_limits_str, directions_str,
        mujoco_names_str;

    int num_joints;

    if (!nh.getParam("model_path", model_path))
    {
        std::cerr << "Failed to get 'model_path' from parameter server." << std::endl;
        return 1;
    }

    if (!nh.getParam("num_joints", num_joints))
    {
        std::cerr << "Failed to get 'num_joints' from parameter server." << std::endl;
        return 1;
    }

    if (!nh.getParam("onnx_path", onnx_path))
    {
        std::cerr << "Failed to get 'onnx_path' from parameter server." << std::endl;
        return 1;
    }

    // 获取所有关节参数
    if (!nh.getParam("joint_names", joint_names_str) || !nh.getParam("kps", kps_str) || !nh.getParam("kds", kds_str) ||
        !nh.getParam("offsets", offsets_str) || !nh.getParam("tau_limits", tau_limits_str) ||
        !nh.getParam("directions", directions_str) || !nh.getParam("joint_names_mujoco", mujoco_names_str))
    {
        ROS_ERROR("Failed to get required parameters from parameter server");
        return -1;
    }

    // 解析参数
    std::vector<std::string> joint_names = parseStringArrayParam(joint_names_str);
    std::vector<double> kps = parseParam(kps_str);
    std::vector<double> kds = parseParam(kds_str);
    std::vector<double> offsets = parseParam(offsets_str);
    std::vector<double> tau_limits = parseParam(tau_limits_str);
    std::vector<double> directions = parseParam(directions_str);
    std::vector<std::string> joint_names_mujoco = parseStringArrayParam(mujoco_names_str);

    ros::Subscriber subSetWalk_ = nh.subscribe<std_msgs::Float32>("/set_walk", 1, setWalkCallback);
    ros::Subscriber subLoadcontroller_ = nh.subscribe<std_msgs::Float32>("/load_controller", 1, loadControllerCallback);
    ros::Subscriber subEmgstop_ = nh.subscribe<std_msgs::Float32>("/emergency_stop", 1, EmergencyStopCallback);
    ros::Subscriber cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, CmdvelCallback);
    ros::Subscriber subWalk2stance_ = nh.subscribe<std_msgs::Float32>("/set_walk2stance", 1, Walk2stanceCallback);
    // 构建joint_params_isaaclab
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
    AMPController amp_controller(onnx_path, num_joints, joint_params_isaaclab, joint_names_mujoco);
    run_real(real_cfg, amp_controller);
    // 进入ROS事件循环
    ros::waitForShutdown();

    return 0;

    // std::vector<AMPController::Cfg::JointParams> joint_params_isaaclab = {
    //     {"left_hip_pitch_joint", 200, 5, -0.1, 200, 1},
    //     {"right_hip_pitch_joint", 200, 5, -0.1, 200, 1},
    //     {"waist_yaw_joint", 150, 6, 0.0, 200, 1},
    //     {"left_hip_roll_joint", 150, 5, 0.0, 200, 1},
    //     {"right_hip_roll_joint", 150, 5, 0.0, 200, 1},
    //     {"waist_roll_joint", 150, 6, 0.0, 200, 1},
    //     {"left_hip_yaw_joint", 150, 5, 0.0, 200, 1},
    //     {"right_hip_yaw_joint", 150, 5, 0.0, 200, 1},
    //     {"waist_pitch_joint", 150, 6, 0.0, 200, 1},
    //     {"left_knee_joint", 200, 5, 0.3, 200, 1},
    //     {"right_knee_joint", 200, 5, 0.3, 200, 1},
    //     {"left_shoulder_pitch_joint", 40, 10, 0.0, 200, 1},
    //     {"right_shoulder_pitch_joint", 40, 10, 0.0, 200, 1},
    //     {"left_ankle_pitch_joint", 20, 2, -0.2, 200, 1},
    //     {"right_ankle_pitch_joint", 20, 2, -0.2, 200, 1},
    //     {"left_shoulder_roll_joint", 40, 10, 0.0, 200, 1},
    //     {"right_shoulder_roll_joint", 40, 10, 0.0, 200, 1},
    //     {"left_ankle_roll_joint", 20, 2, 0.0, 200, 1},
    //     {"right_ankle_roll_joint", 20, 2, 0.0, 200, 1},
    //     {"left_shoulder_yaw_joint", 40, 10, 0.0, 200, 1},
    //     {"right_shoulder_yaw_joint", 40, 10, 0.0, 200, 1},
    //     {"left_elbow_joint", 40, 10, 0.87, 200, 1},
    //     {"right_elbow_joint", 40, 10, 0.87, 200, 1}};

    // std::vector<std::string> joint_names_mujoco = {
    //     "left_hip_pitch_joint",      "left_hip_roll_joint",        "left_hip_yaw_joint",
    //     "left_knee_joint",           "left_ankle_pitch_joint",     "left_ankle_roll_joint",
    //     "right_hip_pitch_joint",     "right_hip_roll_joint",       "right_hip_yaw_joint",
    //     "right_knee_joint",          "right_ankle_pitch_joint",    "right_ankle_roll_joint",
    //     "waist_yaw_joint",           "waist_roll_joint",           "waist_pitch_joint",
    //     "left_shoulder_pitch_joint", "left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    //     "left_elbow_joint",          "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    //     "right_shoulder_yaw_joint",  "right_elbow_joint"};
}