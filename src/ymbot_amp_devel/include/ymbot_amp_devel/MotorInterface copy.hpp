#pragma once
#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include <memory.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
#include "Console.hpp"
#include "command.h"
#include "transmit.h"

#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <controller_manager_msgs/SwitchController.h>
#include <pthread.h>
#include "parallejoint.hpp"

#include "ymbot_amp_devel/SharedMemory.hpp"

#define JOINT_MOTOR_NUM 14
#define PARALLEL_MOTOR_NUM 4 // 并联电机数量
namespace MotorInterface
{

    class MotorController
    {
    public:
        MotorController();
        ~MotorController();
        bool init();
        int sendMotorCommand();  // 发送电机命令
        void receiveMotorData(); // 接收电机数据
        void run();              // 循环运行接口逻辑
        void JointControlMode(double tarJointAngle[JOINT_MOTOR_NUM]);
        SendEncosMotorData sendDataJ[JOINT_MOTOR_NUM];
        void clearMotorCommand();

        void readData(
            RecEncosMotorData recM[JOINT_MOTOR_NUM],
            RecEncosMotorData recJ[JOINT_MOTOR_NUM],
            SendEncosMotorData sendDataM[JOINT_MOTOR_NUM]);

        void read_joint();
        void write_joint();

    private:
        std::thread worker_thread;
        std::ofstream saveTxt;
        SharedMemory shm;
        // std::ofstream motorRecTxt, motorSendTxt;
        int motorTimeCount{0};
        ParalleJoint *parallejoint;
        std::atomic<bool> is_running_; // 控制线程运行状态

        int start_read = 0;
        bool running = true;

        RecEncosMotorData jointDataRec_[JOINT_MOTOR_NUM]{0}, motorDataRec_[JOINT_MOTOR_NUM]{0};
        RecEncosMotorData motorData_previousRec_[JOINT_MOTOR_NUM]{0};
        SendEncosMotorData jointDataSend_[JOINT_MOTOR_NUM]{0}, motorDataSend_[JOINT_MOTOR_NUM]{0};

        // EncosImuData imuData_{};
        // EncosImuData imuTopicSaveData_{};
        int powerLimit_{};
        int contactThreshold_{};

        bool estimateContact_[4];
        // pthread_mutex_t lock, lockImuTopic;
        pthread_mutex_t recDataLock, sendDataLock, yksSendcmdMutex_;
        YKSMotorData yksSendcmdzero_[JOINT_MOTOR_NUM] = {};
        YKSMotorData yksSendcmd_[JOINT_MOTOR_NUM];
        float transform_CurrentPos[JOINT_MOTOR_NUM] = {0};

        const std::vector<int> directionMotor_{
            1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1,
            1, 1};                                              // 电机空间转换A,转换完之后，单腿中间四个关节方向与世界坐标系正方向相反
        const double ankleMotorParallelDir[4] = {1, -1, -1, 1}; // 把电机的方向转为并联算法方向，向上为正
        const double ankleJointParallelDir[4] = {1, 1, 1, -1};  // 把并联的关节角度，转到urdf

        const double baseMotor_[JOINT_MOTOR_NUM] = {0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        // ---------- joint angle pd -------------------------------
        const double default_kp[JOINT_MOTOR_NUM] = {230.0, 70.0, 50.0, 230.0, 180.0, 120.0,
                                                    230.0, 70.0, 50.0, 230.0, 180.0, 120.0,
                                                    250, 250};
        const double default_kd[JOINT_MOTOR_NUM] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                                                    5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
                                                    5, 5};

        // ----------------- system protect ------------------
        // 关节限位保护
        const double maxJointPos[JOINT_MOTOR_NUM] = {1.6, 0.8, 1.5, 1.5, 0.74, 1.5,
                                                     1.6, 0.8, 1.5, 1.5, 0.74, 1.5,
                                                     2, 0.83};

        const double minJointPos[JOINT_MOTOR_NUM] = {-1.6, -0.9, -1.5, -1.45, -0.74, -1.5,
                                                     -1.6, -0.9, -1.5, -1.45, -0.74, -1.5,
                                                     -1.65, -1.3};
        const double maxJointVel[JOINT_MOTOR_NUM] = {15, 15, 14, 15, 12, 12,
                                                     15, 15, 14, 15, 12, 12,
                                                     15, 15};
        const double maxMotorTor[JOINT_MOTOR_NUM] = {94, 94, 120, 150, 70, 70,
                                                     94, 94, 120, 150, 70, 70,
                                                     94, 94};

        const double ankleMotorMaxPos[PARALLEL_MOTOR_NUM] = {0.70, 0.86, 0.97, 0.70};
        const double ankleMotorMinPos[PARALLEL_MOTOR_NUM] = {-0.97, -0.67, -0.73, -0.86};

        const int parallelMotorId[PARALLEL_MOTOR_NUM] = {4, 5, 10, 11};
        double parallelMotorLimit[PARALLEL_MOTOR_NUM] = {1.0, 1.0, 1.0, 1.0};
        int errorState{0};                                            //
        void RecDataProtect(RecEncosMotorData recM[JOINT_MOTOR_NUM],  // rec motor data
                            RecEncosMotorData recJ[JOINT_MOTOR_NUM]); // rec joint data

        std::chrono::high_resolution_clock::time_point initTime;
    };

} // namespace legged

#endif // MOTOR_INTERFACE_H