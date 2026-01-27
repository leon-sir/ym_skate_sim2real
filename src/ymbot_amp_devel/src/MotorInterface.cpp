#include "ymbot_amp_devel/MotorInterface.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <condition_variable>
#include <atomic>
#include <ctime>
#include <algorithm>
// #define ANKLE_PD_

using Clock = std::chrono::high_resolution_clock;
#define RL_MODEL

namespace MotorInterface
{
    MotorController::MotorController() : is_running_(false), parallejoint(nullptr), shm(true)
    {
    }

    MotorController::~MotorController()
    {
        is_running_ = false; // 停止线程
        running = false;     // 设置运行标志为false
    }

    double getValidValue(double currentValue, double &previousValue)
    {
        if (std::isnan(currentValue))
        {
            // 当前值为NaN，返回上一次有效值
            std::cerr << "```````````` currentValue ```````NAN       " << std::endl;
            return previousValue;
        }
        else
        {
            // 当前值有效，更新并返回
            previousValue = currentValue;
            return currentValue;
        }
    }

    double deadVel(double input, double dead)
    {
        double dout = 0;
        if (input < dead && input > 0)
            dout = input + dead;
        else if (input < 0 && input > -dead)
            dout = input + dead;
        else
            dout = input;

        return dout;
    }

    void MotorController::read_joint()
    {
        shm.readJointData(sendDataJ);
    }
    void MotorController::write_joint()
    {
        shm.writeJointData(jointDataRec_);
    }

    bool MotorController::init()
    {
        int ec_slavecount;
        parallejoint = new ParalleJoint(0.065, 0.121, 0.055, 0.080, 0.136, 0.070, 0.015);
        std::string package_path = ".";
        saveTxt.open(package_path + "/dataMotor.txt");

        try
        {
            //  std::cerr << " ``````````````````` 000   " << std::endl;
            ec_slavecount = EtherCAT_Init("enp2s0");
            // std::cerr << " ``````````````````` 000   " << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Caught standard exception: " << e.what() << std::endl;
            throw;
        }
        // catch (...)
        // {
        //     std::cerr << "Caught unknown exception" << std::endl;
        //     throw;
        // }

        std::cerr << "```````````````````开始EtherCAT初始化" << std::endl;
        if (ec_slavecount <= 0)
        {
            std::cerr << "```````````````````未找到从站，程序退出" << std::endl;
            return false;
        }

        for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
        {
            yksSendcmd_[i].pos_des_ = 0;
            yksSendcmd_[i].vel_des_ = 0;
            yksSendcmd_[i].kp_ = 0;
            yksSendcmd_[i].kd_ = 0;
            yksSendcmd_[i].ff_ = 0;
        }
        std::cerr << "```````````````````EtherCAT初始化完成" << std::endl;
        return true;
    }

    void MotorController::receiveMotorData()
    {
        EtherCAT_Get_State();
        if (start_read > 100)
        {
            // pthread_mutex_lock(&recDataLock);
            // std::cout << "joint ";
            for (int i = 0; i < JOINT_MOTOR_NUM; i++)
            {
                motorDataRec_[i].pos_ = getValidValue(motorDate_recv[i].pos_, motorData_previousRec_[i].pos_);
                motorDataRec_[i].vel_ = getValidValue(motorDate_recv[i].vel_, motorData_previousRec_[i].vel_);
                motorDataRec_[i].tau_ = getValidValue(motorDate_recv[i].tau_, motorData_previousRec_[i].tau_);

                motorData_previousRec_[i].pos_ = motorDataRec_[i].pos_;
                motorData_previousRec_[i].vel_ = motorDataRec_[i].vel_;
                motorData_previousRec_[i].tau_ = motorDataRec_[i].tau_;
                if (i != 4 && i != 5 &&
                    i != 10 && i != 11)
                {
                    jointDataRec_[i].pos_ = (motorDataRec_[i].pos_ - baseMotor_[i]) * directionMotor_[i];
                    jointDataRec_[i].vel_ = motorDataRec_[i].vel_ * directionMotor_[i];
                    jointDataRec_[i].tau_ = motorDataRec_[i].tau_ * directionMotor_[i];
                }
                //    std::cout<< " joint data  " << i << " q " << motorDataRec_[i].pos_<<std::endl;
                // std::cout << i << " " << motorDataRec_[i].pos_ << " ";
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            Eigen::Vector2d angleJointLeft,
                ankleMotor;
            angleJointLeft.setZero();
            // pitch roll
            ankleMotor << motorDataRec_[4].pos_ * ankleMotorParallelDir[0],
                motorDataRec_[5].pos_ * ankleMotorParallelDir[1];
            // if (ankleMotor[1] < -0.8)
            //     ankleMotor[1] = -0.8;
            // std::cout << " ankleMotor[0].pos_  " << ankleMotor[0]
            //           << " rankleMotor " << ankleMotor[1] << std::endl;

            parallejoint->frame(0);
            angleJointLeft = parallejoint->fw(jointDataRec_[4].pos_ * ankleJointParallelDir[0],
                                              jointDataRec_[5].pos_ * ankleJointParallelDir[1],
                                              ankleMotor[0], ankleMotor[1]);
            // std::cout << ankleMotor[0] << " " << ankleMotor[1] << "  0- " << angleJointLeft[0] << " 1-  " << angleJointLeft[1] << std::endl;
            // std::cout << motorDataRec_[4].pos_ << " " << motorDataRec_[5].pos_ << "  0- " << angleJointLeft[0] << " 1-  " << angleJointLeft[1] << std::endl;

            Eigen::Vector2d motorVel_readL(motorDataRec_[4].vel_ * ankleMotorParallelDir[0],
                                           motorDataRec_[5].vel_ * ankleMotorParallelDir[1]);
            Eigen::Vector2d motorForce_readL(motorDataRec_[4].tau_ * ankleMotorParallelDir[0],
                                             motorDataRec_[5].tau_ * ankleMotorParallelDir[1]);
            Eigen::Vector2d jointVel_readL = Eigen::Vector2d::Zero();
            Eigen::Vector2d jointForce_readL = Eigen::Vector2d::Zero();
            parallejoint->forceVelMotor2Joint(angleJointLeft(0), angleJointLeft(1),
                                              ankleMotor[0], ankleMotor[1],
                                              jointVel_readL, jointForce_readL,
                                              motorVel_readL, motorForce_readL);

            jointDataRec_[4].pos_ = angleJointLeft(0) * ankleJointParallelDir[0];
            jointDataRec_[5].pos_ = angleJointLeft(1) * ankleJointParallelDir[1];
            jointDataRec_[4].vel_ = jointVel_readL(0) * ankleJointParallelDir[0];
            jointDataRec_[5].vel_ = jointVel_readL(1) * ankleJointParallelDir[1];
            jointDataRec_[4].tau_ = jointForce_readL(0) * ankleJointParallelDir[0];
            jointDataRec_[5].tau_ = jointForce_readL(1) * ankleJointParallelDir[1];
            // std::cout << jointDataRec_[4].pos_ << "  jointDataRec_[4].pos rr_   " << std::endl;
            // std::cout << jointDataRec_[5].pos_ << "  jointDataRec_[5].pos rr_   " << std::endl;
            Eigen::Vector2d angleRight, ankleMotorRight;
            angleRight.setZero();
            ankleMotorRight << motorDataRec_[10].pos_ * ankleMotorParallelDir[2], motorDataRec_[11].pos_ * ankleMotorParallelDir[3];
            // if (ankleMotorRight[1] < -0.8)
            //     ankleMotorRight[1] = -0.8;
            parallejoint->frame(0);
            angleRight = parallejoint->fw(jointDataRec_[10].pos_ * ankleJointParallelDir[2],
                                          jointDataRec_[11].pos_ * ankleJointParallelDir[3],
                                          ankleMotorRight[0], ankleMotorRight[1]);
            // std::cout << ankleMotorRight[0] << " " << ankleMotorRight[1] << "  0- " << angleRight[0] << " 1-  " << angleRight[1] << std::endl;
            // std::cout << motorDataRec_[10].pos_ << " " << motorDataRec_[11].pos_ << "  0- " << angleRight[0] << " 1-  " << angleRight[1] << std::endl;

            Eigen::Vector2d motorVel_readR(motorDataRec_[10].vel_ * ankleMotorParallelDir[2],
                                           motorDataRec_[11].vel_ * ankleMotorParallelDir[3]);
            Eigen::Vector2d motorForce_readR(motorDataRec_[10].tau_ * ankleMotorParallelDir[2],
                                             motorDataRec_[11].tau_ * ankleMotorParallelDir[3]);
            Eigen::Vector2d jointVel_readR = Eigen::Vector2d::Zero();
            Eigen::Vector2d jointForce_readR = Eigen::Vector2d::Zero();
            parallejoint->forceVelMotor2Joint(angleRight(0), angleRight(1),
                                              ankleMotorRight[0], ankleMotorRight[1],
                                              jointVel_readR, jointForce_readR,
                                              motorVel_readR, motorForce_readR);
            jointDataRec_[10].pos_ = angleRight(0) * ankleJointParallelDir[2];
            jointDataRec_[11].pos_ = angleRight(1) * ankleJointParallelDir[3];
            jointDataRec_[10].vel_ = jointVel_readR(0) * ankleJointParallelDir[2];
            jointDataRec_[11].vel_ = jointVel_readR(1) * ankleJointParallelDir[3];
            jointDataRec_[10].tau_ = jointForce_readR(0) * ankleJointParallelDir[2];
            jointDataRec_[11].tau_ = jointForce_readR(1) * ankleJointParallelDir[3];
            // std::cout << jointDataRec_[10].pos_ << "  jointDataRec_[10].pos rr_   " << std::endl;
            // std::cout << jointDataRec_[11].pos_ << "  jointDataRec_[11].pos rr_   " << std::endl;
            //---------------------------------------data recerive protect ----------------------------------------------------
            auto t2 = std::chrono::high_resolution_clock::now();
            auto dt_read = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            // if (dt_read > 500)
            //     std::cout << "read_joint: " << dt_read << " us " << std::endl;

            auto tmpTime = Clock::now();
            std::chrono::duration<double> time_span2 = std::chrono::duration_cast<std::chrono::duration<double>>(tmpTime - initTime);
            ros::Duration elapsedTime_2 = ros::Duration(time_span2.count());

            saveTxt << elapsedTime_2 << ',';
            for (int i = 0; i < JOINT_MOTOR_NUM; i++)
            {
                saveTxt << motorDataRec_[i].pos_ << ',';
            }
            for (int i = 0; i < JOINT_MOTOR_NUM; i++)
            {
                saveTxt << motorDataRec_[i].vel_ << ',';
            }
            for (int i = 0; i < JOINT_MOTOR_NUM; i++)
            {
                saveTxt << motorDataRec_[i].tau_ << ',';
            }
            for (int i = 0; i < JOINT_MOTOR_NUM; i++)
            {
                saveTxt << jointDataRec_[i].pos_ << ',';
            }
            for (int i = 0; i < JOINT_MOTOR_NUM; i++)
            {
                saveTxt << jointDataRec_[i].vel_ << ',';
            }

            for (int i = 0; i < JOINT_MOTOR_NUM; i++)
            {
                saveTxt << jointDataRec_[i].tau_ << ',';
                // std::cout << " joint data  " << i << " q " << jointDataRec_[i].pos_ << std::endl;
            }

            saveTxt << std::endl;
            // //---------------------------------------data recerive protect ----------------------------------------------------

            RecDataProtect(motorDataRec_, jointDataRec_);
            // pthread_mutex_unlock(&recDataLock);
        }
        else
            start_read++;
    }

    void MotorController::JointControlMode(double tarJointAngle[JOINT_MOTOR_NUM])
    {
        SendEncosMotorData sendDataJ[JOINT_MOTOR_NUM];
        for (int i = 0; i < JOINT_MOTOR_NUM; i++)
        {
            sendDataJ[i].pos_des_ = tarJointAngle[i];
            sendDataJ[i].vel_des_ = 0;
            sendDataJ[i].ff_ = 0;
            sendDataJ[i].kp_ = default_kp[i];
            sendDataJ[i].kd_ = default_kd[i];
            // std::cout<< " ii " << i << " Kp" << sendDataJ[i].kp_ << std::endl;
        }

        // sendMotorCommand(sendDataJ);
        sendMotorCommand();
    }

    int MotorController::sendMotorCommand()
    {
        static double lastFF[4] = {0};
        static int start_time = 0;
        if (errorState > 0)
        {
            running = false;
            // std::cout << " running  false, error state " << errorState << std::endl;
        }

        double pitchL = jointDataRec_[4].pos_ * ankleJointParallelDir[0];
        double rollL = jointDataRec_[5].pos_ * ankleJointParallelDir[1];
        Eigen::Vector2d angleMotorLeft{motorDataRec_[4].pos_ * ankleMotorParallelDir[0],
                                       motorDataRec_[5].pos_ * ankleMotorParallelDir[1]}; //
                                                                                          //
        double pitchR = jointDataRec_[10].pos_ * ankleJointParallelDir[2];
        double rollR = jointDataRec_[11].pos_ * ankleJointParallelDir[3];
        Eigen::Vector2d angleMotorRight = {motorDataRec_[10].pos_ * ankleMotorParallelDir[2],
                                           motorDataRec_[11].pos_ * ankleMotorParallelDir[3]};

        if (running == true && start_time > 100)
        {
            auto t3 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
            {
                if (i != 4 && i != 5 &&
                    i != 10 && i != 11)
                {
                    motorDataSend_[i].pos_des_ = (sendDataJ[i].pos_des_ * directionMotor_[i] + baseMotor_[i]);
                    motorDataSend_[i].vel_des_ = sendDataJ[i].vel_des_ * directionMotor_[i];
                    motorDataSend_[i].ff_ = sendDataJ[i].ff_ * directionMotor_[i];
                    motorDataSend_[i].kp_ = sendDataJ[i].kp_;
                    motorDataSend_[i].kd_ = sendDataJ[i].kd_;
                }
            }
            Eigen::Matrix2d JcInvL;
            Eigen::Matrix2d JcL = parallejoint->Jac(pitchL, rollL, angleMotorLeft[0], angleMotorLeft[1], JcInvL);
            // Eigen::Vector2d angleMotorLeft = parallejoint->ik(pitchL, rollL);
            // Eigen::Matrix2d JcL = parallejoint->Jac(pitchL, rollL, angleMotorLeft[0], angleMotorLeft[1], JcInvL);
            // std::cout << jointDataRec_[5].pos_ << "  joi3ntDat arallelDir_   " << std::endl;

            double sj4 = sendDataJ[4].pos_des_ * ankleJointParallelDir[0];
            // if (sj4 < -0.6)
            //     sj4 = -0.6;
            Eigen::Vector2d dprL{sendDataJ[5].pos_des_ * ankleJointParallelDir[1] - rollL, // rooll  pitch
                                 sj4 - pitchL};
            // m1n,m2n = parTest.ik(pitch,roll)
            // Jc = parTest.Jac(pitch,roll,m1n,m2n)
            // dd = Jc @ np.array([dr,dp])
            Eigen::Vector2d ddL = JcL * dprL;
            Eigen::Matrix2d aa = JcInvL * JcInvL.transpose();

            motorDataSend_[4].pos_des_ = (angleMotorLeft[0] + ddL(0)) * ankleMotorParallelDir[0];
            motorDataSend_[5].pos_des_ = (angleMotorLeft[1] + ddL(1)) * ankleMotorParallelDir[1];
            motorDataSend_[4].vel_des_ = 0;
            motorDataSend_[5].vel_des_ = 0;
            motorDataSend_[4].ff_ = 0;
            motorDataSend_[5].ff_ = 0;
            motorDataSend_[4].kp_ = aa(0, 0) * sendDataJ[5].kp_ + aa(0, 1) * sendDataJ[4].kp_;
            motorDataSend_[5].kp_ = aa(1, 0) * sendDataJ[5].kp_ + aa(1, 1) * sendDataJ[4].kp_;

            motorDataSend_[4].kd_ = aa(0, 0) * sendDataJ[5].kd_ + aa(0, 1) * sendDataJ[4].kd_;
            motorDataSend_[5].kd_ = aa(1, 0) * sendDataJ[5].kd_ + aa(1, 1) * sendDataJ[4].kd_;

            // if (motorDataSend_[5].pos_des_ > 0.8)
            //     motorDataSend_[5].pos_des_ = 0.8;
            // if (motorDataSend_[4].pos_des_ < -0.8)
            //     motorDataSend_[4].pos_des_ = -0.8;

            // Eigen::Vector2d angleMotorRight = parallejoint->ik(pitchR, rollR);

            Eigen::Matrix2d JcInvR;
            Eigen::Matrix2d JcR = parallejoint->Jac(pitchR, rollR, angleMotorRight[0], angleMotorRight[1], JcInvR);
            double sj10 = sendDataJ[10].pos_des_ * ankleJointParallelDir[2];
            // if (sj10 < -0.6)
            //     sj10 = -0.6;
            Eigen::Vector2d dprR{sendDataJ[11].pos_des_ * ankleJointParallelDir[3] - rollR, // rooll  pitch
                                 sj10 - pitchR};
            // m1n,m2n = parTest.ik(pitch,roll)
            // Jc = parTest.Jac(pitch,roll,m1n,m2n)
            // dd = Jc @ np.array([dr,dp])
            Eigen::Vector2d ddR = JcR * dprR;
            Eigen::Matrix2d bb = JcInvR * JcInvR.transpose();

            motorDataSend_[10].pos_des_ = (angleMotorRight[0] + ddR(0)) * ankleMotorParallelDir[2];
            motorDataSend_[11].pos_des_ = (angleMotorRight[1] + ddR(1)) * ankleMotorParallelDir[3];
            motorDataSend_[10].vel_des_ = 0;
            motorDataSend_[11].vel_des_ = 0;
            motorDataSend_[10].ff_ = 0;
            motorDataSend_[11].ff_ = 0;
            motorDataSend_[10].kp_ = bb(0, 0) * sendDataJ[11].kp_ + bb(0, 1) * sendDataJ[10].kp_;
            motorDataSend_[11].kp_ = bb(1, 0) * sendDataJ[11].kp_ + bb(1, 1) * sendDataJ[10].kp_;
            motorDataSend_[10].kd_ = bb(0, 0) * sendDataJ[11].kd_ + bb(0, 1) * sendDataJ[10].kd_;
            motorDataSend_[11].kd_ = bb(1, 0) * sendDataJ[11].kd_ + bb(1, 1) * sendDataJ[10].kd_;

            auto t4 = std::chrono::high_resolution_clock::now();
            auto dt_send = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();

            if (dt_send > 500)
                std::cout << "send_joint: " << dt_send << " us " << std::endl;
        }

        else
        {
            for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
            {
                motorDataSend_[i].pos_des_ = 0;
                motorDataSend_[i].vel_des_ = 0;
                motorDataSend_[i].ff_ = 0;
                motorDataSend_[i].kp_ = 0;
                motorDataSend_[i].kd_ = 0;
            }
        }

        start_time += 1;
        for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
        {
            if (motorDataSend_[i].ff_ > maxMotorTor[i])
                motorDataSend_[i].ff_ = maxMotorTor[i];
            else if (motorDataSend_[i].ff_ < -maxMotorTor[i])
                motorDataSend_[i].ff_ = -maxMotorTor[i];
        }

        for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
        {
            yksSendcmd_[i].pos_des_ = motorDataSend_[i].pos_des_;
            yksSendcmd_[i].vel_des_ = motorDataSend_[i].vel_des_;
            yksSendcmd_[i].ff_ = motorDataSend_[i].ff_;
            yksSendcmd_[i].kp_ = motorDataSend_[i].kp_;
            yksSendcmd_[i].kd_ = motorDataSend_[i].kd_;
        }

        EtherCAT_Send_Command((YKSMotorData *)yksSendcmd_);
        return errorState;
    }
    void MotorController::clearMotorCommand()
    {
        // pthread_mutex_lock(&yksSendcmdMutex_);

        for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
        {
            yksSendcmd_[i].pos_des_ = 0;
            yksSendcmd_[i].vel_des_ = 0;
            yksSendcmd_[i].kp_ = 0;
            yksSendcmd_[i].kd_ = 10;
            yksSendcmd_[i].ff_ = 0;
        }
    }
    // rec motor data
    void MotorController::readData(
        RecEncosMotorData recM[JOINT_MOTOR_NUM],
        RecEncosMotorData recJ[JOINT_MOTOR_NUM],
        SendEncosMotorData sendDataM[JOINT_MOTOR_NUM])
    {
        // ------------------- rec data--------------
        for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
        {
            recM[i] = motorDataRec_[i];
            recJ[i] = jointDataRec_[i];
        }
        for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
        {
            sendDataM[i] = motorDataSend_[i];
        }
    }

    // 数据保护
    void MotorController::RecDataProtect(RecEncosMotorData recM[JOINT_MOTOR_NUM], // rec motor data
                                         RecEncosMotorData recJ[JOINT_MOTOR_NUM]) // rec joint data
    {
        // 关节限位保护
        for (int i = 0; i < JOINT_MOTOR_NUM; i++)
        {
            if (recJ[i].pos_ > maxJointPos[i] ||
                recJ[i].pos_ < minJointPos[i]) //||
            // abs(recJ[i].vel_) > maxJointVel[i]
            {
                if (errorState == 0 && i != 4 && i != 5 && i != 10 && i != 11)
                {
                    errorState = i;
                    std::cout << " rec joint data error   " << i << " q " << recJ[i].pos_
                              << " dq " << recJ[i].vel_ << " tor  " << recJ[i].tau_ << std::endl;
                    // std::cout << " recJ[4].pos_  " << recM[4].pos_
                    //           << " recJ[5].pos_ " << recM[5].pos_ << std::endl;
                }
            }

            if (abs(recM[i].tau_) > maxMotorTor[i] * 1.5)
            {
                // 使用去抖：只有连续超过阈值次数才置错误状态
                torqueExceedCount[i]++;
                if (torqueExceedCount[i] >= torqueExceedThreshold)
                {
                    if (errorState == 0)
                    {
                        errorState = i + 50;
                        std::cout << " rec Motor Tor data error   " << errorState << " tor  " << recM[i].tau_
                                  << " pos " << recM[i].pos_ << " vel " << recM[i].vel_ << " count " << torqueExceedCount[i] << std::endl;
                    }
                }
                else
                {
                    std::cout << " rec Motor Tor warning   " << i << " tor " << recM[i].tau_ << " count " << torqueExceedCount[i] << std::endl;
                }
            }
            else
            {
                // 未超过阈值则清零计数
                torqueExceedCount[i] = 0;
            }
            if (abs(recM[i].vel_) > maxJointVel[i] * 1.5)
            {
                if (errorState == 0)
                {
                    errorState = i + 100;
                    std::cout << " rec maxJointVel data error   " << errorState << " tor  " << recM[i].vel_ << std::endl;
                }
            }
        }

        for (int i = 0; i < PARALLEL_MOTOR_NUM; i++)
        {
            // std::cout << " parallel  Motor  i   " << i << "   "
            //           << recM[parallelMotorId[i]].pos_ << "\n";
            if (recM[parallelMotorId[i]].pos_ > ankleMotorMaxPos[i] ||
                recM[parallelMotorId[i]].pos_ < ankleMotorMinPos[i] ||
                abs(recM[parallelMotorId[i]].tau_) > maxMotorTor[parallelMotorId[i]] * torqueThresholdMultiplier)
            {
                if (errorState == 0)
                {
                    // errorState = parallelMotorId[i] + 200;
                    std::cout << " parallel  Motor  error   " << parallelMotorId[i]
                              << " motor q " << recM[parallelMotorId[i]].pos_
                              << " parallelMotorLimit[i] " << parallelMotorLimit[i] << std::endl;
                }
                // 阻止指数衰减到接近0，设置下限
                parallelMotorLimit[i] = std::max(parallelMotorLimit[i] * 0.95, parallelMotorLimitMin);
            }
            else
            {
                if (parallelMotorLimit[i] < 1.0)
                {
                    parallelMotorLimit[i] += 0.002;
                }
            }
        }
    }

    void MotorController::run()
    {
        is_running_ = true;
        // auto cycle_duration_motor = std::chrono::milliseconds(1);
        // auto next_time_point_motor = std::chrono::steady_clock::now() + cycle_duration_motor;
        // auto loop_start = std::chrono::high_resolution_clock::now();
        using Clock = std::chrono::steady_clock;
        using namespace std::chrono;

        // === 设置周期 ===
        const auto cycle_duration = milliseconds(1); // 1ms 周期

        // === 初始化时间点 ===
        auto next_time_point = Clock::now() + cycle_duration;

        // jointDataRec_[4].pos_ = 0;
        // jointDataRec_[5].pos_ = 0;
        // jointDataRec_[10].pos_ = 0;
        // jointDataRec_[11].pos_ = 0;

        while (is_running_)
        {
            auto loop_start = Clock::now();
            // static int count = 0;
            // std::cout << "looptimes: " << count << std::endl;
            // auto dt_cycle = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - loop_start).count();
            // loop_start = std::chrono::high_resolution_clock::now();
            auto t1 = std::chrono::high_resolution_clock::now();
            read_joint();
            // std::cout << "read_joint: " <<std::endl;
            auto t2 = std::chrono::high_resolution_clock::now();
            sendMotorCommand();
            // 提取目标角度
            // double tarJointAngle[JOINT_MOTOR_NUM];
            // for (int i = 0; i < JOINT_MOTOR_NUM; ++i)
            // {
            //     tarJointAngle[i] = sendDataJ[i].pos_des_;
            // }
            // JointControlMode(tarJointAngle);
            auto t3 = std::chrono::high_resolution_clock::now();
            receiveMotorData();
            auto t4 = std::chrono::high_resolution_clock::now();
            write_joint();
            auto t5 = std::chrono::high_resolution_clock::now();

            // auto loop_end = std::chrono::high_resolution_clock::now();

            auto dt_read = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
            auto dt_send = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
            auto dt_receive = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
            auto dt_write = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
            // auto dt_total = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();

            // std::cout << "read_joint: " << dt_read << " us, send: " << dt_send
            //           << " us, receive: " << dt_receive << " us, write: " << dt_write
            //           << " us, total: " << std::endl; //<< dt_total << " us" << "  dt_cycle " << dt_cycle
            // count++;
            // === 统计耗时 ===
            auto loop_end = Clock::now();
            auto loop_duration = duration_cast<microseconds>(loop_end - loop_start).count();

            // === 漂移检测 ===
            if (loop_end > next_time_point + cycle_duration)
            {
                std::cerr << "[WARN] Loop overrun detected! "
                          << "Duration: " << loop_duration << " us, "
                          << "Missed cycle(s): "
                          << duration_cast<milliseconds>(loop_end - next_time_point).count() / cycle_duration.count()
                          << std::endl;

                // === 【恢复】：把 next_time_point 重置为当前时间 + 周期
                next_time_point = loop_end + cycle_duration;
            }
            else
            {
                // 正常累加周期
                next_time_point += cycle_duration;
            }

            // === 睡眠到下一个时间点 ===
            std::this_thread::sleep_until(next_time_point);
            // std::this_thread::sleep_until(next_time_point_motor);
            // next_time_point_motor += cycle_duration_motor;
        }
        std::cout << "endl motorinterface running " << std::endl;
        saveTxt.close();
    }
} // namespace legged

int main(int argc, char **argv)
{
    // 绑定 CPU 核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(8, &cpuset); // 绑定到 CPU 核心5
    CPU_SET(9, &cpuset); // 绑定到 CPU 核心6

    pid_t pid = getpid(); // main线程对应进程
    if (sched_setaffinity(pid, sizeof(cpu_set_t), &cpuset) == -1)
    {
        perror("sched_setaffinity");
    }
    else
    {
        std::cout << "Successfully set CPU affinity to core 12" << std::endl;
    }

    // 设置线程优先级
    sched_param sch_params;
    sch_params.sched_priority = 80; // 取值范围通常是 1-99

    if (sched_setscheduler(0, SCHED_FIFO, &sch_params))
    {
        perror("sched_setscheduler");
    }
    else
    {
        std::cout << "Successfully set priority to 10" << std::endl;
    }
    MotorInterface::MotorController motorcontroller;
    if (!motorcontroller.init())
    {
        std::cerr << "Failed to initialize MotorController." << std::endl;
        // return -1; // 如果初始化失败，退出程序
    }
    motorcontroller.run();
    return 0;
}
