#include "ymbot_amp_devel/eumotor_interface.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "ymbot_amp_devel/SharedMemoryArm.hpp"
#include <csignal>
#include <atomic>

std::atomic<bool> exit_flag(false);

void sigint_handler(int sig)
{
    std::cout << "\n捕获到 Ctrl+C,准备安全退出..." << std::endl;
    exit_flag = true;
}

MotorData motor_data;
std::mutex data_mutex;
// #################################16关节(7*2+2)##########################################
//  double euMaxPos[JOINT_ARM_NUMBER] = {1.74, 3.14, 4.88, 3.14, 1.57, 1.5, 1.5,
//                                       1.74, 0.34, 3.14, 3.14, 1.57, 1.5, 1.5,
//                                       1.5, 1.5};
//  double euMinPos[JOINT_ARM_NUMBER] = {-6, -0.34, -5, -0.34, -1.57, -1.5, -1.5,
//                                       -6, -3.14, -1.95, -0.34, -1.57, -1.5, -1.5,
//                                       -1.5, -1.5};
//  double euDir[JOINT_ARM_NUMBER] = {1, 1, -1, 1, 1, 1, 1,
//                                    -1, 1, -1, -1, 1, 1,1,
//                                    1,1};
//  double euBase[JOINT_ARM_NUMBER] = {0.0, -0.2, 0, 0.5, 0, 0, 0,
//                                     0.0, 0.2, 0, 0.5, 0, 0, 0,
//                                     0.0, 0.0};
// #################################12关节(5*2+2)##########################################
// double euMaxPos[JOINT_ARM_NUMBER] = {1.74, 3.14, 4.88, 3.14, 1.57,
//                                      1.74, 1.34, 3.14, 3.14, 1.57,
//                                      1.5, 1.5};
// double euMinPos[JOINT_ARM_NUMBER] = {-6, -1.34, -5, -1.50, -1.57,
//                                      -6, -3.14, -1.95, -1.50, -1.57,
//                                      -1.5, -1.5};
                                     
double euMaxPos[JOINT_ARM_NUMBER] = {1.74, 3.14, 4.88, 3.14, 1.57,
                                     1.74, 1.34, 3.14, 3.14, 1.57};
double euMinPos[JOINT_ARM_NUMBER] = {-6, -1.34, -5, -1.50, -1.57,
                                     -6, -3.14, -1.95, -1.50, -1.57};                                    
// double euMinPos[JOINT_ARM_NUMBER] = {-6, -0.34, -5, -0.34, -1.57,
// -6, -3.14, -1.95, -0.34, -1.57,
// -1.5, -1.5};
// double euDir[JOINT_ARM_NUMBER] = {1, 1, 1, 1, 1,
//                                   1, 1, 1, 1, 1,
//                                   1, 1};
double euDir[JOINT_ARM_NUMBER] = {1, 1, 1, 1, 1,
                                  1, 1, 1, 1, 1};
// double euDir[JOINT_ARM_NUMBER] = {1, 1, -1, 1, 1,
// -1, 1, -1, -1, 1,
// 1,1};
// double euBase[JOINT_ARM_NUMBER] = {0.0, -0.2, 0, 0.5, 0,
//                                    0.0, 0.2, 0, 0.5, 0,
//                                    0.0, 0.0};

double euBase[JOINT_ARM_NUMBER] = {0.0, 0.0, 0, 0., 0,
                                   0.0, 0.0, 0, 0., 0};

std::vector<MotorGroupConfig> load_motor_config(const std::string &config_path)
{
    std::vector<MotorGroupConfig> groups;
    YAML::Node config = YAML::LoadFile(config_path);

    for (const auto &group_node : config["motor_groups"])
    {
        MotorGroupConfig group;
        group.can_index = group_node["can_index"].as<int>();
        group.motor_ids = group_node["ids"].as<std::vector<int>>();
        groups.push_back(group);
    }
    return groups;
}

// 单例实现
MotorInterface &MotorInterface::getInstance()
{
    static MotorInterface instance;
    return instance;
}

// 修改 eumotor_interface.cpp 中的构造函数
MotorInterface::MotorInterface() : shmArm(true)
{
    // 从配置文件加载配置
    auto motor_groups = load_motor_config("/home/pc/ymbot_e_23dofwalk/src/ymbot_amp_devel/config/motor_config.yaml");
    // 创建所有电机实例
    for (const auto &group : motor_groups)
    {
        for (int motor_id : group.motor_ids)
        {
            YmbotJointEu motor;
            motor.motor_id = motor_id;
            motor.dev_index = group.can_index; // 直接使用配置的CAN口
            motors.push_back(motor);
        }
    }
}

// MotorInterface::MotorInterface() : shmArm(true) {
//     // 初始化电机ID配置
//     // const int id_array[] = {41, 42, 43, 44, 45};         //右臂
//     const int id_array[] = {31, 32, 33, 34, 35};       //左臂
//     motors.resize(sizeof(id_array)/sizeof(id_array[0]));

//     for(size_t i=0; i<motors.size(); ++i){
//         motors[i].motor_id = id_array[i];

//         // 分配CAN接口
//         if(id_array[i] > 40 && id_array[i] < 50) motors[i].dev_index = 2;
//         else if(id_array[i] > 30 && id_array[i] < 40) motors[i].dev_index = 0;
//         else if(id_array[i] > 10 && id_array[i] < 30) motors[i].dev_index = 1;
//         else  {std::cout << "There is an id number in the id_array that does not match the actual motor." << std::endl;
//                std::exit(EXIT_FAILURE); // 立即退出程序
//               //   return;
//               }
//     }
// }

// 根据索引获取电机实际ID
int MotorInterface::getMotorID(size_t index)
{
    auto &instance = getInstance();
    if (index < instance.motors.size())
    {
        return instance.motors[index].motor_id;
    }
    return -1; // 无效索引
}

void MotorInterface::disable()
{
    auto &instance = getInstance();
    instance.running = false;

    // 禁用所有电机
    for (auto &m : instance.motors)
    {
        m.motor_disabled();
    }

    // 关闭CAN通信
    for (int i = 0; i < instance.n_motor_group; ++i)
    {
        planet_freeDLL(i);
    }
}

void MotorInterface::runThread()
{
    getInstance().mainLoop();
}

bool saftCheck(EuArmData dataRes_[JOINT_ARM_NUMBER])
{
    for (size_t i = 0; i < JOINT_ARM_NUMBER; ++i)
    {
        if (dataRes_[i].pos_ > euMaxPos[i] || dataRes_[i].pos_ < euMinPos[i])
        {
            std::cout << "eu pos err id : " << i << "  pos  " << dataRes_[i].pos_ << "\n";
            return false;
        }
    }
    return true;
}

void MotorInterface::mainLoop()
{
    initMotors();
    auto loop_start = std::chrono::steady_clock::now();
    double maxDelta = 0.5;
    double init_des[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    shmArm.writeJointDatatoMotorArm(init_des);

    while (running)
    {
        auto t0 = std::chrono::steady_clock::now();

        if (exit_flag)
        {
            auto t_exit_start = std::chrono::steady_clock::now();
            std::cout << "检测到退出信号..." << std::endl;
            moveToHomePosition(); // 先回零
            auto t_exit_mid = std::chrono::steady_clock::now();
            std::cout << "正在安全关闭电机..." << std::endl;
            disable();
            auto t_exit_end = std::chrono::steady_clock::now();
            std::cout << "moveToHomePosition耗时: "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(t_exit_mid - t_exit_start).count() << " ms, "
                      << "disable耗时: "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(t_exit_end - t_exit_mid).count() << " ms"
                      << std::endl;
            break;
        }

        // auto t1 = std::chrono::steady_clock::now();
        receiveFeedback();
        // auto t2 = std::chrono::steady_clock::now();

        for (size_t i = 0; i < motors.size(); ++i)
        {
            dataRes_[i].pos_ = (motor_data.current_position_rad[i] - M_PI) * euDir[i] + euBase[i];
            dataRes_[i].vel_ = motor_data.current_velocity_rads[i];
        }
        // auto t3 = std::chrono::steady_clock::now();

        running = saftCheck(dataRes_);
        // auto t4 = std::chrono::steady_clock::now();

        shmArm.writeJointDataArm(dataRes_);
        // auto t5 = std::chrono::steady_clock::now();

        shmArm.readJointDataArm(pos_des_arm_);
        // auto t6 = std::chrono::steady_clock::now();

        for (size_t i = 0; i < motors.size(); ++i)
        {
            double delta = ((pos_des_arm_[i] - euBase[i]) * euDir[i] + M_PI) - motor_data.current_position_rad[i];
            if (delta > maxDelta)
                delta = maxDelta;
            else if (delta < -maxDelta)
                delta = -maxDelta;
            motor_data.move_rad[i] = delta;
        }
        // auto t7 = std::chrono::steady_clock::now();

        sendCommands();
        // auto t8 = std::chrono::steady_clock::now();

        // 精确周期控制
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - loop_start);
        if (elapsed.count() < 20)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20 - elapsed.count()));
        }
        // auto t9 = std::chrono::steady_clock::now();

        // // 打印各步骤耗时
        // std::cout << "[Timing] receiveFeedback: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us, "
        //           << "dataRes赋值: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() << "us, "
        //           << "saftCheck: " << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << "us, "
        //           << "writeJointDataArm: " << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() << "us, "
        //           << "readJointDataArm: " << std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() << "us, "
        //           << "delta计算: " << std::chrono::duration_cast<std::chrono::microseconds>(t7 - t6).count() << "us, "
        //           << "sendCommands: " << std::chrono::duration_cast<std::chrono::microseconds>(t8 - t7).count() << "us, "
        //           << "sleep: " << std::chrono::duration_cast<std::chrono::microseconds>(t9 - t8).count() << "us"
        //           << std::endl;

        loop_start = std::chrono::steady_clock::now();
    }

    disable();
}

// void MotorInterface::mainLoop() {
//     initMotors();
//     auto loop_start = std::chrono::steady_clock::now();
//     double maxDelta = 0.5; // 0.1---5 rad/s
//     // double init_des[12] = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};
//     // double init_des[16] = {0, 0, 0, 0, 0, 0, 0,
//     //                        0, 0, 0, 0, 0, 0, 0,
//     //                        0, 0};
//     double init_des[12] = {0, 0, 0, 0, 0,
//                            0, 0, 0, 0, 0,
//                            0, 0};
//     shmArm.writeJointDatatoMotorArm(init_des);
//     // 50Hz控制循环
//     while (running)
//     {
//         if (exit_flag) {
//             std::cout << "检测到退出信号..." << std::endl;
//             moveToHomePosition(); // 先回零
//             std::cout << "正在安全关闭电机..." << std::endl;
//             disable();
//             break;
//         }

//         auto elapsed_microseconds =
//             std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - loop_start);
//         std::cout << "elapsed_microseconds ----  time: " << elapsed_microseconds.count() << "us\n";
//         loop_start = std::chrono::steady_clock::now();

//         receiveFeedback();
//         for (size_t i = 0; i < motors.size(); ++i)
//         {
//             dataRes_[i].pos_ = (motor_data.current_position_rad[i] - M_PI) * euDir[i] + euBase[i];
//             dataRes_[i].vel_ = motor_data.current_velocity_rads[i];
//             // std::cout << " " << i << " " << dataRes_[i].pos_;
//         }
//         // std::cout << "\n";
//         running = saftCheck(dataRes_);
//         shmArm.writeJointDataArm(dataRes_);

//         shmArm.readJointDataArm(pos_des_arm_);
//         for (size_t i = 0; i < motors.size(); ++i)
//         {
//             std::cout << "pos_des_arm_: " << i << " " << pos_des_arm_[i] << "\n";
//             double delta = ((pos_des_arm_[i] - euBase[i]) * euDir[i] + M_PI) - motor_data.current_position_rad[i];
//             if (delta > maxDelta)
//             {
//                 delta = maxDelta;
//             }
//             else if (delta < -maxDelta)
//             {
//                 delta = -maxDelta;
//             }
//             motor_data.move_rad[i] = delta;
//             std::cout << "motor_data.move_rad[i]: "<< i << " " << delta << "\n";
//         }
//         // std::cout << "pos_des_arm_: " << pos_des_arm_[0]
//         //           << " dataRes_[i].pos_: " << dataRes_[0].pos_
//         //           << "motor_data.move_rad: " << motor_data.move_rad[0]
//         //           << "\n";
//         sendCommands();

//         // 精确周期控制
//         auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
//             std::chrono::steady_clock::now() - loop_start);
//         std::cout << "eu ----  time: " << elapsed.count() << "ms\n";
//         if (elapsed.count() < 20)
//         {
//             std::cout << "！！！！！！！！！！！！！\n";
//             std::this_thread::sleep_for(std::chrono::milliseconds(20 - elapsed.count()));
//         }
//     }

//     disable();
// }

void MotorInterface::moveToHomePosition()
{
    std::cout << "正在回归初始位置..." << std::endl;
    const double home_rad = home_position_deg * M_PI / 180.0;
    const double step = 0.3; // 每次最大移动步长（弧度），可根据实际调整
    bool all_reached = false;

    while (!all_reached)
    {
        all_reached = true;
        for (size_t i = 0; i < motors.size(); ++i)
        {
            double current = motor_data.current_position_rad[i];
            double delta = home_rad - current;
            if (fabs(delta) > 0.01)
            { // 允许1度误差
                all_reached = false;
                double move = std::max(std::min(delta, step), -step);
                motor_data.move_rad[i] = move;
            }
            else
            {
                motor_data.move_rad[i] = 0.0;
            }
        }
        sendCommands();
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
        receiveFeedback();
    }
    std::cout << "已回归初始位置。" << std::endl;
}

void MotorInterface::initMotors()
{
    // 初始化CAN总线
    for (int dev = 0; dev < n_motor_group; ++dev)
    {
        if (planet_initDLL(planet_DeviceType_Canable, dev, 0, planet_Baudrate_1000) != PLANET_SUCCESS)
        { // CAN_SUCCESS
            std::cerr << "CAN" << dev << " initialization failed!" << std::endl;
            disable();
            std::exit(EXIT_FAILURE); // 立即退出程序
            // return;
        }
    }

    // 初始化电机
    for (auto &motor : motors)
    {
        if (!motor.motor_initialization_CSP())
        {
            std::cerr << "Motor " << motor.motor_id << " initialization failed!" << std::endl;
            disable();
            std::exit(EXIT_FAILURE); // 立即退出程序
            // return;
        }
        smoothHoming(motor);
    }

    // 初始化数据结构
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        motor_data.move_rad.resize(motors.size(), 0.0); // 初始增量为0
        motor_data.current_position_rad.resize(motors.size());
        motor_data.current_velocity_rads.resize(motors.size());
    }
}

void MotorInterface::smoothHoming(YmbotJointEu &motor)
{
    const double target_deg = 180.0;
    float current_deg = 0.0;

    // 渐进式归位
    do
    {
        planet_getPosition(motor.dev_index, motor.motor_id, &current_deg);
        double step = (target_deg - current_deg) * 0.5; // 50%步进
        // 打印调试信息
        // std::cout << "Motor ID: " << motor.motor_id
        //           << " | Current Position: " << current_deg << "°"
        //           << " | Target Position: " << current_deg + step << "°"
        //           << std::endl;
        planet_quick_setTargetPosition(motor.dev_index, motor.motor_id, current_deg + step);
        // 快速直接归为180度（零位）
        // planet_quick_setTargetPosition(motor.dev_index, motor.motor_id, target_deg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (fabs(target_deg - current_deg) > 0.5);
}

void MotorInterface::sendCommands()
{
    for (size_t i = 0; i < motors.size(); ++i)
    {
        // 获取当前实际位置（单位：rad）
        double current_rad = motor_data.current_position_rad[i];
        // 计算绝对目标位置：current + delta
        double target_rad = current_rad + motor_data.move_rad[i];

        double target_deg = target_rad * 180.0 / M_PI;
        if (planet_quick_setTargetPosition(motors[i].dev_index, motors[i].motor_id, target_deg) != PLANET_SUCCESS)
        {
            std::cerr << "Motor " << motors[i].motor_id << " send command failed!" << std::endl;
            break;
        }
        motor_data.move_rad[i] = 0.0; // 清空增量
    }
}

void MotorInterface::receiveFeedback()
{
    for (size_t i = 0; i < motors.size(); ++i)
    {
        float deg, rpm;
        if (planet_getPosition(motors[i].dev_index, motors[i].motor_id, &deg) == PLANET_SUCCESS && // CAN_SUCCESS
            planet_getVelocity(motors[i].dev_index, motors[i].motor_id, &rpm) == PLANET_SUCCESS)
        {
            // 单位转换
            motor_data.current_position_rad[i] = deg * M_PI / 180.0;
            motor_data.current_velocity_rads[i] = rpm * (2 * M_PI / 60.0);
            // std::cerr << "velocity data:  " << i << "    :  " << rpm << std::endl;
        }
        else
        {
            std::cerr << "lost One frame of data!" << std::endl;
        }

        // if (planet_getPosition(motors[i].dev_index, motors[i].motor_id, &deg) == PLANET_SUCCESS)
        // {
        //     // 单位转换
        //     motor_data.current_position_rad[i] = deg * M_PI / 180.0;
        //     // motor_data.current_velocity_rads[i] = rpm * 2 * M_PI / 60.0;
        // }
    }
}

int main(int argc, char **argv)
{
    // 绑定 CPU 核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(10, &cpuset);  // 绑定到 CPU 核心9
    CPU_SET(12, &cpuset);  // 绑定到 CPU 核心9
    CPU_SET(14, &cpuset); // 绑定到 CPU 核心9
    CPU_SET(16, &cpuset); // 绑定到 CPU 核心9
    pid_t pid = getpid(); // main线程对应进程
    if (sched_setaffinity(pid, sizeof(cpu_set_t), &cpuset) == -1)
    {
        perror("sched_setaffinity");
    }
    else
    {
        std::cout << "Successfully set CPU affinity to core 9" << std::endl;
    }

    // 设置线程优先级
    sched_param sch_params;
    sch_params.sched_priority = 20; // 取值范围通常是 1-99

    if (sched_setscheduler(0, SCHED_FIFO, &sch_params))
    {
        perror("sched_setscheduler");
    }
    else
    {
        std::cout << "Successfully set priority to 80" << std::endl;
    }
    // 启动 motor 主控制线程
    MotorInterface &motor_interface = MotorInterface::getInstance();
    // std::thread motor_thread(&MotorInterface::mainLoop , &motor_interface);

    // 注册 Ctrl+C 信号处理
    std::signal(SIGINT, sigint_handler);

    motor_interface.running = true;
    motor_interface.mainLoop();
    // 主线程等待子线程退出
    // motor_thread.join();
    return 0;
}