#include "ymbot_amp_devel/amp_controller.hpp"

using namespace std;

AMPController::AMPController(const std::string& onnx_path,
                             const int& num_joints,
                             const std::vector<Cfg::JointParams>& joint_params_isaaclab,
                             const std::vector<std::string>& joint_names_mujoco) {
    // cfg.joint_params_isaaclab = {{"left_hip_pitch_joint", 200, 5, -0.1, 200, 1},
    //                              {"right_hip_pitch_joint", 200, 5, -0.1, 200, 1},
    //                              {"waist_yaw_joint", 150, 6, 0.0, 200, 1},
    //                              {"left_hip_roll_joint", 150, 5, 0.0, 200, 1},
    //                              {"right_hip_roll_joint", 150, 5, 0.0, 200, 1},
    //                              {"waist_roll_joint", 150, 6, 0.0, 200, 1},
    //                              {"left_hip_yaw_joint", 150, 5, 0.0, 200, 1},
    //                              {"right_hip_yaw_joint", 150, 5, 0.0, 200, 1},
    //                              {"waist_pitch_joint", 150, 6, 0.0, 200, 1},
    //                              {"left_knee_joint", 200, 5, 0.3, 200, 1},
    //                              {"right_knee_joint", 200, 5, 0.3, 200, 1},
    //                              {"left_shoulder_pitch_joint", 40, 10, 0.0, 200, 1},
    //                              {"right_shoulder_pitch_joint", 40, 10, 0.0, 200, 1},
    //                              {"left_ankle_pitch_joint", 20, 2, -0.2, 200, 1},
    //                              {"right_ankle_pitch_joint", 20, 2, -0.2, 200, 1},
    //                              {"left_shoulder_roll_joint", 40, 10, 0.0, 200, 1},
    //                              {"right_shoulder_roll_joint", 40, 10, 0.0, 200, 1},
    //                              {"left_ankle_roll_joint", 20, 2, 0.0, 200, 1},
    //                              {"right_ankle_roll_joint", 20, 2, 0.0, 200, 1},
    //                              {"left_shoulder_yaw_joint", 40, 10, 0.0, 200, 1},
    //                              {"right_shoulder_yaw_joint", 40, 10, 0.0, 200, 1},
    //                              {"left_elbow_joint", 40, 10, 0.87, 200, 1},
    //                              {"right_elbow_joint", 40, 10, 0.87, 200, 1}};


    // cfg.joint_names_mujoco = {"left_hip_pitch_joint",      "left_hip_roll_joint",        "left_hip_yaw_joint",
    //                           "left_knee_joint",           "left_ankle_pitch_joint",     "left_ankle_roll_joint",
    //                           "right_hip_pitch_joint",     "right_hip_roll_joint",       "right_hip_yaw_joint",
    //                           "right_knee_joint",          "right_ankle_pitch_joint",    "right_ankle_roll_joint",
    //                           "waist_yaw_joint",           "waist_roll_joint",           "waist_pitch_joint",
    //                           "left_shoulder_pitch_joint", "left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    //                           "left_elbow_joint",          "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    //                           "right_shoulder_yaw_joint",  "right_elbow_joint"};


    cfg.env.num_joints = num_joints;
    //gsj
    cfg.env.num_single_obs = num_joints * 3 + 6;
    cfg.mjc2lab.resize(num_joints);
    obs_ = Eigen::VectorXd::Zero(cfg.env.num_single_obs);
    actions_ = Eigen::VectorXd::Zero(cfg.env.num_joints);

    cfg.joint_params_isaaclab = joint_params_isaaclab;
    cfg.joint_names_mujoco = joint_names_mujoco;

    // 检查参数一致性
    if (cfg.joint_params_isaaclab.size() != cfg.env.num_joints || cfg.joint_names_mujoco.size() != cfg.env.num_joints) {
        std::cerr << "Error: Joint parameters size mismatch. Expected " << cfg.env.num_joints << " joints, but got:\n"
                  << "joint_params_isaaclab: " << cfg.joint_params_isaaclab.size() << "\n"
                  << "joint_names_mujoco: " << cfg.joint_names_mujoco.size() << std::endl;
        throw std::runtime_error("Joint parameters size mismatch");
    }

    // 计算索引
    for (size_t mujoco_idx = 0; mujoco_idx < cfg.joint_names_mujoco.size(); ++mujoco_idx) {
        const std::string& mujoco_name = cfg.joint_names_mujoco[mujoco_idx];

        // 使用标准迭代器而非指针算术
        auto it = std::find_if(cfg.joint_params_isaaclab.begin(), cfg.joint_params_isaaclab.end(),
                               [&mujoco_name](const AMPController::Cfg::JointParams& jp) {
                                   return std::string(jp.joint_name) == mujoco_name;
                               });

        if (it != cfg.joint_params_isaaclab.end()) {
            size_t isaaclab_idx = std::distance(cfg.joint_params_isaaclab.begin(), it);

            // 添加边界检查
            if (mujoco_idx < cfg.mjc2lab.size()) {
                cfg.mjc2lab[mujoco_idx] = isaaclab_idx;
            }
            else {
                std::cerr << "Warning: Index out of bounds in joint mapping (" << isaaclab_idx << ", " << mujoco_idx
                          << ")" << std::endl;
            }
        }
        else {
            std::cerr << "Warning: Joint name '" << mujoco_name << "' not found in joint_params_isaaclab" << std::endl;
        }
    }

    // ----------------------- mjc2lab -----------------------
    // 0 3 6 9 13 17 1 4 7 10 14 18 2 5 8 11 15 19 21 12 16 20 22

    // // 输出调试信息
    // cout << "----------------------- lab2mjc ------------------------\n";
    // for (const auto& idx : cfg.lab2mjc) {
    //     cout << idx << " ";
    // }
    // cout << endl;

    // cout << "----------------------- mjc2lab -----------------------\n";
    // for (const auto& idx : cfg.mjc2lab) {
    //     cout << idx << " ";
    // }
    // cout << endl;


    // Initialize ONNX Runtime session
    Ort::Env env(ORT_LOGGING_LEVEL_INFO, "ONNXRuntimeModel");
    Ort::SessionOptions session_options;

    // Specify CPU provider
    session_options.SetIntraOpNumThreads(16);

    try {
        onnx_session_ = std::make_shared<Ort::Session>(env, onnx_path.c_str(), session_options);
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to load ONNX model: " << e.what() << std::endl;
        throw;
    }
}

Eigen::VectorXd AMPController::quaternion_to_euler(const double x, const double y, const double z, const double w) {
    // 滚转（x 轴旋转）
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll_x = std::atan2(t0, t1);

    // 俯仰（y 轴旋转）
    double t2 = +2.0 * (w * y - z * x);
    t2 = std::clamp(t2, -1.0, 1.0); // 限制 t2 在 -1 到 1 之间
    double pitch_y = std::asin(t2);

    // 偏航（z 轴旋转）
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = std::atan2(t3, t4);

    // 返回滚转、俯仰、偏航的向量（以弧度表示）
    Eigen::VectorXd euler_angle(3);
    euler_angle << roll_x, pitch_y, yaw_z;


    // Adjust yaw by subtracting the first Z-axis angle after it has been recorded
    if (first_z_angle_ == std::numeric_limits<double>::infinity()) {
        first_z_angle_ = yaw_z; // Store the first Z-axis angle
    }
    else {
        euler_angle[2] -= first_z_angle_; // Subtract the first Z-axis angle from subsequent ones
    }

    // Adjust angles to keep them in the range [-pi, pi]
    for (int i = 0; i < euler_angle.size(); ++i) {
        if (euler_angle[i] > M_PI) {
            euler_angle[i] -= 2 * M_PI;
        }
        else if (euler_angle[i] < -M_PI) {
            euler_angle[i] += 2 * M_PI;
        }
    }

    return euler_angle;
}

// gsj
void AMPController::onnx_output(const Eigen::VectorXd& projected_gravity,
                                const Eigen::VectorXd& velocity_commands,
                                const Eigen::VectorXd& joint_pos,
                                const Eigen::VectorXd& joint_vel) {
    for (size_t i = 0; i < projected_gravity.size(); ++i) {
        obs_[i] = projected_gravity[i];
    } // 3
    for (size_t i = 0; i < velocity_commands.size(); ++i) {
        obs_[3 + i] = velocity_commands[i];
    } // 3
    for (size_t i = 0; i < joint_pos.size(); ++i) {
        obs_[6 + i] = joint_pos[i] - cfg.joint_params_isaaclab[i].offset;
    }
    for (size_t i = 0; i < joint_vel.size(); ++i) {
        obs_[cfg.env.num_joints + 6 + i] = joint_vel[i] * cfg.control.velocity_scale;
    };
    for (size_t i = 0; i < actions_.size(); ++i) {
        obs_[cfg.env.num_joints * 2 + 6 + i] = actions_[i];
    }

// void AMPController::onnx_output(const Eigen::VectorXd& base_linear_velocity,
//                                const Eigen::VectorXd& base_angular_velocity,
//                                const Eigen::VectorXd& projected_gravity,
//                                const Eigen::VectorXd& velocity_commands,
//                                const Eigen::VectorXd& joint_pos,
//                                const Eigen::VectorXd& joint_vel) {
//     // 构造 obs
//     for (size_t i = 0; i < base_linear_velocity.size(); ++i) {
//         obs_[i] = base_linear_velocity[i];
//     } // 3
//     for (size_t i = 0; i < base_angular_velocity.size(); ++i) {
//         obs_[base_linear_velocity.size() + i] = base_angular_velocity[i];
//     } // 3
//     for (size_t i = 0; i < projected_gravity.size(); ++i) {
//         obs_[base_angular_velocity.size() + i] = projected_gravity[i];
//     } // 3
//     for (size_t i = 0; i < velocity_commands.size(); ++i) {
//         obs_[projected_gravity.size() + i] = velocity_commands[i];
//     } // 3
//     for (size_t i = 0; i < joint_pos.size(); ++i) {
//         obs_[velocity_commands.size() + i] = joint_pos[i];
//     } // 22
//     for (size_t i = 0; i < joint_vel.size(); ++i) {
//         obs_[joint_pos.size() + i] = joint_vel[i];
//     } // 22
//     for (size_t i = 0; i < actions_.size(); ++i) {
//         obs_[joint_vel.size() + i] = actions_[i];
//     } // 22


    // Clip observations
    for (size_t i = 0; i < obs_.size(); ++i) {
        obs_[i] = std::clamp(obs_[i], -cfg.control.clip_observations, cfg.control.clip_observations);
    }

    // 直接使用当前帧的 obs 作为策略输入
    Eigen::VectorXd policy_input = obs_;

    // Convert policy input to float32 and prepare input for ONNX Runtime
    std::vector<float> policy_input_vec(policy_input.data(), policy_input.data() + policy_input.size());
    std::vector<int64_t> input_shape = {1, cfg.env.num_single_obs}; // 修改输入形状

    // Prepare input tensor
    std::vector<Ort::Value> inputs;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, policy_input_vec.data(), policy_input_vec.size(), input_shape.data(), input_shape.size());
    inputs.push_back(std::move(input_tensor));


    // Run inference
    std::vector<const char*> input_node_names = {"obs"};
    std::vector<const char*> output_node_names = {"actions"};
    auto output_tensors = onnx_session_->Run(Ort::RunOptions{nullptr}, input_node_names.data(), inputs.data(),
                                             inputs.size(), output_node_names.data(), output_node_names.size());

    // 处理输出
    float* output_data = output_tensors[0].GetTensorMutableData<float>();

    for (int i = 0; i < cfg.env.num_joints; ++i) {
        actions_[i] = output_data[i];
    }

    // cout << "onnx_actions: ";
    // for (size_t i = 0; i < actions_.size(); ++i) {
    //     cout << actions_[i] << " ";
    // }
    // cout << endl;
    // cout << endl;
    // cout << endl;
}


Eigen::VectorXd AMPController::pd_output(const Eigen::VectorXd& target_q,
                                         const Eigen::VectorXd& q,
                                         const Eigen::VectorXd& target_dq,
                                         const Eigen::VectorXd& dq) {
    // 确保输入向量的大小相同
    std::size_t n = target_q.size();
    Eigen::VectorXd torque(n);

    for (std::size_t i = 0; i < n; i++) {
        torque[i] = (target_q[i] - q[i]) * cfg.joint_params_isaaclab[i].kp +
                    (target_dq[i] - dq[i]) * cfg.joint_params_isaaclab[i].kd;
    }

    return torque;
}
