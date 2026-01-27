#ifndef EUMOTOR_INTERFACE_H
#define EUMOTOR_INTERFACE_H

#include <vector>
#include <atomic>
#include <mutex>
#include "ymbot_amp_devel/eu_planet.h"
#include "ymbot_amp_devel/ymbot_joint_eu.h"
// 在 eumotor_interface.hpp 中添加
#include <yaml-cpp/yaml.h>
#include "ymbot_amp_devel/SharedMemoryArm.hpp"

struct MotorGroupConfig
{
    int can_index;
    std::vector<int> motor_ids;
};

// 外部数据交换结构体
struct MotorData
{
    std::vector<double> move_rad;              // 输入：移动弧度（rad）
    std::vector<double> current_position_rad;  // 输出：当前位置（rad）
    std::vector<double> current_velocity_rads; // 输出：当前速度（rad/s）
};

extern MotorData motor_data;
extern std::mutex data_mutex;
// extern std::atomic<bool> exit_flag;

class MotorInterface
{
public:
    // MotorInterface();
    static void disable();
    void moveToHomePosition();
    static void runThread();
    void mainLoop();
    static int getMotorID(size_t index);
    // 获取电机总数
    static size_t getMotorCount()
    {
        return getInstance().motors.size();
    }
    bool running{true};
    static MotorInterface &getInstance();
    EuArmData dataRes_[JOINT_ARM_NUMBER];
    double pos_des_arm_[JOINT_ARM_NUMBER];

private:
    SharedMemoryArm shmArm;
    MotorInterface();
    void initMotors();
    void sendCommands();
    void receiveFeedback();
    void smoothHoming(YmbotJointEu &motor);

    std::vector<YmbotJointEu> motors;
    const int n_motor_group = 3;
    const double home_position_deg = 180.0;
};

#endif