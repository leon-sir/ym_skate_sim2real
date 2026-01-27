#ifndef AMP_CONTROLLER_H
#define AMP_CONTROLLER_H

#include <Eigen/Dense>
#include <deque>
#include <string>
#include <algorithm> // for std::clamp
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <onnxruntime_cxx_api.h>
#include <memory>
#include "phase_generator.hpp"

// 声明控制器类,包含控制器所需的参数配置和变量
class AMPController
{
public:
    // 参数配置
    class Cfg
    {
    public:
        struct Env
        {
            int num_joints;     // 机器人的关节数量
            int num_single_obs; // 单个观测值的数量
        };

        struct Control
        {   
            double lin_vel_scale = 0.2;
            double anglar_vel_scale = 0.25;
            double projected_gravity = 1.0;
            double velocity_command_scale = 1.0;
            double joint_pos_scale = 1.0;
            double joint_vel_scale = 1.0;
            double actions_ = 1.0;

            double velocity_scale = 0.05;
            


            double clip_lin_vel = 100.0;
            double clip_anglar_vel = 100.0;
            double clip_projected_gravity = 100.0;
            double clip_velocity_command = 100.0;
            double clip_joint_pos = 100.0;
            double clip_joint_vel = 100.0;
            double clip_observations = 100.0;
            double clip_actions = 100.0;

            int observation_length = 10;

            // 添加LSTM配置参数
            int lstm_num_layers = 1;
            int lstm_hidden_size = 256;
        };

        struct JointParams
        {
            std::string joint_name;
            double kp;
            double kd;
            double offset;
            double tau_limit;
            int direction;
        };

        Env env;
        Control control;
        std::vector<JointParams> joint_params_isaaclab;
        std::vector<std::string> joint_names_mujoco;
        std::vector<size_t> mjc2lab;

        Cfg()
        {
            // 构造函数体 - mjc2lab将在AMPController构造函数中手动resize
        }
    };

    Cfg cfg; // 添加 Cfg 成员变量

    // 除参数配置以外,需要交互和改变的变量
    Eigen::VectorXd actions_;
    double first_z_angle_ = std::numeric_limits<double>::infinity(); // To store the first Z-axis angle

    std::shared_ptr<Ort::Env> onnx_env_;  // Must persist for the lifetime of the session
    std::shared_ptr<Ort::Session> onnx_session_;
    Eigen::VectorXd obs_;
    std::deque<Eigen::VectorXd> obs_buffer_;
    PhaseGenerator phase_generator_;

    // 添加LSTM状态管理所需成员变量
    Eigen::MatrixXd h_in_;
    Eigen::MatrixXd c_in_;

    // 添加log文件流
    std::ofstream obs_log_file_;

    AMPController(const std::string &onnx_path,
                  const int &num_joints,
                  const std::vector<Cfg::JointParams> &joint_params_isaaclab,
                  const std::vector<std::string> &joint_names_mujoco);

    Eigen::VectorXd quaternion_to_euler(const double x, const double y, const double z, const double w);
    // gsj
    // void onnx_output(const Eigen::VectorXd &projected_gravity,
    //                  const Eigen::VectorXd &rotation_velocity,
    //                  const Eigen::VectorXd &velocity_commands,
    //                  const Eigen::VectorXd &joint_pos,
    //                  const Eigen::VectorXd &joint_vel);

    void onnx_output(const Eigen::VectorXd& base_linear_velocity,
                     const Eigen::VectorXd& base_angular_velocity,
                     const Eigen::VectorXd& projected_gravity,
                     const Eigen::VectorXd& velocity_commands,
                     const Eigen::VectorXd& joint_pos,
                     const Eigen::VectorXd& joint_vel,
                     const Eigen::VectorXd& action,
                     const Eigen::VectorXd& phase);

    Eigen::VectorXd pd_output(const Eigen::VectorXd &target_q,
                              const Eigen::VectorXd &q,
                              const Eigen::VectorXd &target_dq,
                              const Eigen::VectorXd &dq);
};

#endif // RL_CONTROLLER_H