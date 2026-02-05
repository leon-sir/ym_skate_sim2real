#include "ymbot_amp_devel/amp_controller.hpp"
#include <fstream>
#include <mutex>
#include <cmath>
#include <ctime>
#include <chrono>
#include <iomanip>

using namespace std;

// [新增] 获取带毫秒的时间戳字符串
std::string get_timestamp_str() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "[%Y-%m-%d %H:%M:%S") 
       << "." << std::setfill('0') << std::setw(3) << ms.count() << "]";
    return ss.str();
}

// 添加全局互斥锁用于ONNX推理
static std::mutex onnx_mutex;

AMPController::AMPController(const std::string &onnx_path,
                             const int &num_joints,
                             const std::vector<Cfg::JointParams> &joint_params_isaaclab,
                             const std::vector<std::string> &joint_names_mujoco)
{
    std::cerr << "AMPController constructor started" << std::endl;
    std::cerr << "onnx_path: " << onnx_path << std::endl;
    std::cerr << "num_joints: " << num_joints << std::endl;
    std::cerr << "joint_params_isaaclab.size(): " << joint_params_isaaclab.size() << std::endl;
    std::cerr << "joint_names_mujoco.size(): " << joint_names_mujoco.size() << std::endl;
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
    // gsj - 修正观测向量大小：3(lin_vel) + 3(ang_vel) + 3(proj_grav) + 3(cmd_vel) + 13(joint_pos) + 13(joint_vel) + 13(actions) + 6(phase) = 57
    cfg.env.num_single_obs = num_joints * 3 + 18;
    std::cerr << "AMPController: num_joints=" << num_joints << ", num_single_obs=" << cfg.env.num_single_obs << std::endl;
    
    cfg.mjc2lab.resize(num_joints);
    std::cerr << "AMPController: mjc2lab resized to " << num_joints << std::endl;
    
    obs_ = Eigen::VectorXd::Zero(cfg.env.num_single_obs);
    std::cerr << "AMPController: obs_ created with size " << cfg.env.num_single_obs << std::endl;
    
    actions_ = Eigen::VectorXd::Zero(cfg.env.num_joints);
    std::cerr << "AMPController: actions_ created with size " << cfg.env.num_joints << std::endl;

    cfg.joint_params_isaaclab = joint_params_isaaclab;
    cfg.joint_names_mujoco = joint_names_mujoco;

    // 初始化LSTM隐藏状态和细胞状态
    h_in_ = Eigen::MatrixXd::Zero(cfg.control.lstm_num_layers, cfg.control.lstm_hidden_size);
    c_in_ = Eigen::MatrixXd::Zero(cfg.control.lstm_num_layers, cfg.control.lstm_hidden_size);

    // 打开log文件用于记录obs数据
    std::string log_filename = "/home/pc/ymbot_e_13dof_skate/obs_log.txt";
    obs_log_file_.open(log_filename, std::ios::out | std::ios::app);
    if (obs_log_file_.is_open()) {
        std::cerr << "Observation log file opened: " << log_filename << std::endl;
        // 写入文件头
        obs_log_file_ << "=== Observation Log Started ===" << std::endl;
        obs_log_file_ << "Timestamp: " << std::time(nullptr) << std::endl;
        obs_log_file_ << "Observation size: " << cfg.env.num_single_obs << std::endl;
        obs_log_file_ << "================================" << std::endl;
        obs_log_file_.flush();
    } else {
        std::cerr << "Failed to open observation log file: " << log_filename << std::endl;
    }

    // 检查参数一致性
    if (cfg.joint_params_isaaclab.size() != cfg.env.num_joints || cfg.joint_names_mujoco.size() != cfg.env.num_joints)
    {
        std::cerr << "Error: Joint parameters size mismatch. Expected " << cfg.env.num_joints << " joints, but got:\n"
                  << "joint_params_isaaclab: " << cfg.joint_params_isaaclab.size() << "\n"
                  << "joint_names_mujoco: " << cfg.joint_names_mujoco.size() << std::endl;
        throw std::runtime_error("Joint parameters size mismatch");
    }

    // 计算索引
    for (size_t mujoco_idx = 0; mujoco_idx < cfg.joint_names_mujoco.size(); ++mujoco_idx)
    {
        const std::string &mujoco_name = cfg.joint_names_mujoco[mujoco_idx];

        // 使用标准迭代器而非指针算术
        auto it = std::find_if(cfg.joint_params_isaaclab.begin(), cfg.joint_params_isaaclab.end(),
                               [&mujoco_name](const AMPController::Cfg::JointParams &jp)
                               {
                                   return std::string(jp.joint_name) == mujoco_name;
                               });

        if (it != cfg.joint_params_isaaclab.end())
        {
            size_t isaaclab_idx = std::distance(cfg.joint_params_isaaclab.begin(), it);

            // 添加边界检查
            if (mujoco_idx < cfg.mjc2lab.size())
            {
                cfg.mjc2lab[mujoco_idx] = isaaclab_idx;
            }
            else
            {
                std::cerr << "Warning: Index out of bounds in joint mapping (" << isaaclab_idx << ", " << mujoco_idx
                          << ")" << std::endl;
            }
        }
        else
        {
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
    // CRITICAL: Ort::Env must persist for the lifetime of the session
    onnx_env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_ERROR, "ONNXRuntimeModel");
    Ort::SessionOptions session_options;

    // 使用最简单的配置
    session_options.SetIntraOpNumThreads(1);  // 单线程
    session_options.SetInterOpNumThreads(1);  // 单线程
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);  // 禁用所有优化
    session_options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);  // 顺序执行

    // 检查ONNX文件是否存在
    std::ifstream onnx_file(onnx_path);
    if (!onnx_file.good()) {
        std::cerr << "ONNX model file not found: " << onnx_path << std::endl;
        throw std::runtime_error("ONNX model file not accessible");
    }
    onnx_file.close();

    try
    {
        onnx_session_ = std::make_shared<Ort::Session>(*onnx_env_, onnx_path.c_str(), session_options);
        
        // 获取模型的输入输出信息
        size_t num_input_nodes = onnx_session_->GetInputCount();
        size_t num_output_nodes = onnx_session_->GetOutputCount();
        
        std::cerr << "ONNX model loaded successfully:" << std::endl;
        std::cerr << "  Input nodes: " << num_input_nodes << std::endl;
        std::cerr << "  Output nodes: " << num_output_nodes << std::endl;
        
        // 获取输入节点信息
        if (num_input_nodes > 0) {
            Ort::AllocatorWithDefaultOptions allocator;
            auto input_name = onnx_session_->GetInputNameAllocated(0, allocator);
            auto input_type_info = onnx_session_->GetInputTypeInfo(0);
            auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
            auto input_shape = input_tensor_info.GetShape();
            
            std::cerr << "  Input[0] name: " << input_name.get() << std::endl;
            std::cerr << "  Input[0] shape: [";
            for (size_t i = 0; i < input_shape.size(); ++i) {
                std::cerr << input_shape[i];
                if (i < input_shape.size() - 1) std::cerr << ", ";
            }
            std::cerr << "]" << std::endl;
        }
        
        // 获取输出节点信息
        if (num_output_nodes > 0) {
            Ort::AllocatorWithDefaultOptions allocator;
            auto output_name = onnx_session_->GetOutputNameAllocated(0, allocator);
            auto output_type_info = onnx_session_->GetOutputTypeInfo(0);
            auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
            auto output_shape = output_tensor_info.GetShape();
            
            std::cerr << "  Output[0] name: " << output_name.get() << std::endl;
            std::cerr << "  Output[0] shape: [";
            for (size_t i = 0; i < output_shape.size(); ++i) {
                std::cerr << output_shape[i];
                if (i < output_shape.size() - 1) std::cerr << ", ";
            }
            std::cerr << "]" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to load ONNX model: " << e.what() << std::endl;
        throw;
    }
}

Eigen::VectorXd AMPController::quaternion_to_euler(const double x, const double y, const double z, const double w)
{
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
    if (first_z_angle_ == std::numeric_limits<double>::infinity())
    {
        first_z_angle_ = yaw_z; // Store the first Z-axis angle
    }
    else
    {
        euler_angle[2] -= first_z_angle_; // Subtract the first Z-axis angle from subsequent ones
    }

    // Adjust angles to keep them in the range [-pi, pi]
    for (int i = 0; i < euler_angle.size(); ++i)
    {
        if (euler_angle[i] > M_PI)
        {
            euler_angle[i] -= 2 * M_PI;
        }
        else if (euler_angle[i] < -M_PI)
        {
            euler_angle[i] += 2 * M_PI;
        }
    }

    return euler_angle;
}

// gsj
void AMPController::onnx_output(const Eigen::VectorXd& base_linear_velocity,
                                const Eigen::VectorXd& base_angular_velocity,
                                const Eigen::VectorXd& projected_gravity,
                                const Eigen::VectorXd& velocity_commands,
                                const Eigen::VectorXd& joint_pos,
                                const Eigen::VectorXd& joint_vel,
                                const Eigen::VectorXd& action,
                                const Eigen::VectorXd& phase)
{
    // 控制调试输出的开关
    static bool debug_input = true;
    static int call_count = 0;
    call_count++;
    
    // 写入log文件而不是终端
    if (obs_log_file_.is_open()) {
        obs_log_file_ << get_timestamp_str() << " " << "\n=== ONNX Input Construction (Call #" << call_count << ") ===" << std::endl;
    }
    
    for (size_t i = 0; i < base_linear_velocity.size(); ++i)
    {
        obs_[i] = base_linear_velocity[i] * cfg.control.lin_vel_scale;
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << i << "] = base_linear_velocity[" << i << "] * lin_vel_scale = " 
                      << base_linear_velocity[i] << " * " << cfg.control.lin_vel_scale << " = " << obs_[i] << std::endl;
        }
    } // 3
    
    for (size_t i = 0; i < base_angular_velocity.size(); ++i)
    {
        obs_[3 + i] = base_angular_velocity[i] * cfg.control.anglar_vel_scale;
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << (3 + i) << "] = base_angular_velocity[" << i << "] * anglar_vel_scale = " 
                      << base_angular_velocity[i] << " * " << cfg.control.anglar_vel_scale << " = " << obs_[3 + i] << std::endl;
        }
    } // 3
    
    for (size_t i = 0; i < projected_gravity.size(); ++i)
    {
        obs_[6 + i] = projected_gravity[i];
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << (6 + i) << "] = projected_gravity[" << i << "] = " << projected_gravity[i] << std::endl;
        }
    } // 3
    
    for (size_t i = 0; i < velocity_commands.size(); ++i)
    {
        obs_[9 + i] = velocity_commands[i]* cfg.control.velocity_commands_scale;
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << (9 + i) << "] = velocity_commands[" << i << "] = " << velocity_commands[i] << std::endl;
        }
    } // 3
    
    if (obs_log_file_.is_open()) {
        obs_log_file_ << get_timestamp_str() << " " << "Joint positions (offset corrected):" << std::endl;
    }
    for (size_t i = 0; i < joint_pos.size(); ++i)
    {
        if (i >= cfg.joint_params_isaaclab.size()) {
            std::cerr << "ONNX: joint_pos index " << i << " out of bounds (joint_params_isaaclab size: " 
                      << cfg.joint_params_isaaclab.size() << ")" << std::endl;
            return;
        }
        obs_[12 + i] = joint_pos[i] - cfg.joint_params_isaaclab[i].offset;
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << (12 + i) << "] = joint_pos[" << i << "] - offset = " 
                      << joint_pos[i] << " - " << cfg.joint_params_isaaclab[i].offset << " = " << obs_[12 + i] << std::endl;
        }
    }
    
    if (obs_log_file_.is_open()) {
        obs_log_file_ << get_timestamp_str() << " " << "Joint velocities (scaled):" << std::endl;
    }
    for (size_t i = 0; i < joint_vel.size(); ++i)
    {
        obs_[25 + i] = joint_vel[i] * cfg.control.velocity_scale;
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << (25 + i) << "] = joint_vel[" << i << "] * velocity_scale = " 
                      << joint_vel[i] << " * " << cfg.control.velocity_scale << " = " << obs_[25 + i] << std::endl;
        }
    };
    
    if (obs_log_file_.is_open()) {
        obs_log_file_ << get_timestamp_str() << " " << "Previous actions:" << std::endl;
    }
    for (size_t i = 0; i < actions_.size(); ++i)
    {
        obs_[38 + i] = actions_[i];
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << (38 + i) << "] = actions_[" << i << "] = " << actions_[i] << std::endl;
        }
    };
    
    if (obs_log_file_.is_open()) {
        obs_log_file_ << get_timestamp_str() << " " << "Phase observations (from PhaseGenerator):" << std::endl;
    }
    for (size_t i = 0; i < phase.size() && i < 6; ++i)
    {
        obs_[51 + i] = phase[i];
        if (obs_log_file_.is_open()) {
            obs_log_file_ << get_timestamp_str() << " " << "obs_[" << (51 + i) << "] = phase[" << i << "] = " << phase[i] << std::endl;
        }
    }
    
    if (obs_log_file_.is_open()) {
        obs_log_file_ << get_timestamp_str() << " " << "Before clipping - obs_ summary:" << std::endl;
        obs_log_file_ << get_timestamp_str() << " " << "  obs_ size: " << obs_.size() << std::endl;
        obs_log_file_ << get_timestamp_str() << " " << "  obs_ range: [" << obs_.minCoeff() << ", " << obs_.maxCoeff() << "]" << std::endl;
    }
    
    // Clip observations
    int clipped_count = 0;
    for (size_t i = 0; i < obs_.size(); ++i)
    {
        double original_value = obs_[i];
        obs_[i] = std::clamp(obs_[i], -cfg.control.clip_observations, cfg.control.clip_observations);
        if (original_value != obs_[i]) {
            if (obs_log_file_.is_open()) {
                obs_log_file_ << get_timestamp_str() << " " << "Clipped obs_[" << i << "]: " << original_value << " -> " << obs_[i] << std::endl;
            }
            clipped_count++;
        }
    }
    
    if (obs_log_file_.is_open()) {
        obs_log_file_ << get_timestamp_str() << " " << "After clipping - obs_ summary:" << std::endl;
        obs_log_file_ << get_timestamp_str() << " " << "  obs_ range: [" << obs_.minCoeff() << ", " << obs_.maxCoeff() << "]" << std::endl;
        obs_log_file_ << get_timestamp_str() << " " << "  clipped values: " << clipped_count << std::endl;
        obs_log_file_ << get_timestamp_str() << " " << "=== End ONNX Input Construction ===" << std::endl;
        obs_log_file_.flush();  // 确保数据实时写入
    }

    // === 2. 直接使用当前帧的 obs 作为策略输入 ===
    Eigen::VectorXd policy_input = obs_;

    // Convert policy input to float32 and prepare input for ONNX Runtime
    std::vector<float> policy_input_vec(policy_input.data(), policy_input.data() + policy_input.size());
    std::vector<int64_t> input_shape = {1, cfg.env.num_single_obs};

    // Prepare input tensor
    std::vector<Ort::Value> inputs;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, policy_input_vec.data(), policy_input_vec.size(), input_shape.data(), input_shape.size());
    inputs.push_back(std::move(input_tensor));

    // 添加LSTM状态输入
    // 准备h_in张量
    std::vector<float> h_in_vec(h_in_.data(), h_in_.data() + h_in_.size());
    std::vector<int64_t> h_in_shape = {1, cfg.control.lstm_num_layers, cfg.control.lstm_hidden_size};
    Ort::Value h_in_tensor = Ort::Value::CreateTensor<float>(
        memory_info, h_in_vec.data(), h_in_vec.size(), h_in_shape.data(), h_in_shape.size());
    inputs.push_back(std::move(h_in_tensor));

    // 准备c_in张量
    std::vector<float> c_in_vec(c_in_.data(), c_in_.data() + c_in_.size());
    std::vector<int64_t> c_in_shape = {1, cfg.control.lstm_num_layers, cfg.control.lstm_hidden_size};
    Ort::Value c_in_tensor = Ort::Value::CreateTensor<float>(
        memory_info, c_in_vec.data(), c_in_vec.size(), c_in_shape.data(), c_in_shape.size());
    inputs.push_back(std::move(c_in_tensor));

    // Run inference (with exception handling and output validation)
    std::vector<const char *> input_node_names = {"obs", "h_in", "c_in"};
    std::vector<const char *> output_node_names = {"actions", "h_out", "c_out"};

    std::vector<Ort::Value> output_tensors;
    
    // 检查会话是否有效
    if (!onnx_session_) {
        std::cerr << "ONNX: Session is null!" << std::endl;
        return;
    }
    
    // 使用互斥锁确保线程安全
    std::lock_guard<std::mutex> lock(onnx_mutex);
    
    try
    {
        // 尝试真正的ONNX推理
        std::cerr << "ONNX: Starting real neural network inference" << std::endl;
        
        // 创建RunOptions
        Ort::RunOptions run_options;
        
        // 验证输入数据
        std::cerr << "ONNX: Input validation - obs size: " << obs_.size() 
                  << ", expected: " << cfg.env.num_single_obs << std::endl;
        
        if (obs_.size() != cfg.env.num_single_obs) {
            std::cerr << "ONNX: Input size mismatch!" << std::endl;
            return;
        }
        
        // 检查输入数据是否包含NaN或无穷大
        bool has_invalid_data = false;
        for (size_t i = 0; i < obs_.size(); ++i) {
            if (!std::isfinite(obs_[i])) {
                std::cerr << "ONNX: Invalid data at index " << i << ": " << obs_[i] << std::endl;
                has_invalid_data = true;
                break;
            }
        }
        
        if (has_invalid_data) {
            std::cerr << "ONNX: Input contains invalid data, skipping inference" << std::endl;
            return;
        }
        
        std::cerr << "ONNX: Input validation passed, calling Run()" << std::endl;
        
        // 执行推理
        auto outputs = onnx_session_->Run(run_options, input_node_names.data(), inputs.data(),
                                          inputs.size(), output_node_names.data(), output_node_names.size());
        
        std::cerr << "ONNX: Run completed successfully, got " << outputs.size() << " outputs" << std::endl;
        output_tensors = std::move(outputs);
        
        // 更新LSTM隐藏状态
        if (outputs.size() >= 3) {
            Ort::Value h_out_tensor = std::move(outputs[1]);
            Ort::Value c_out_tensor = std::move(outputs[2]);

            // 获取输出张量维度
            auto h_out_shape = h_out_tensor.GetTensorTypeAndShapeInfo().GetShape();
            auto c_out_shape = c_out_tensor.GetTensorTypeAndShapeInfo().GetShape();

            // 验证维度是否匹配 (batch=1, layers, hidden_size)
            if (h_out_shape.size() == 3 && h_out_shape[0] == 1 && 
                h_out_shape[1] == cfg.control.lstm_num_layers && 
                h_out_shape[2] == cfg.control.lstm_hidden_size &&
                c_out_shape.size() == 3 && c_out_shape[0] == 1 && 
                c_out_shape[1] == cfg.control.lstm_num_layers && 
                c_out_shape[2] == cfg.control.lstm_hidden_size) {

                float* h_out_data = h_out_tensor.GetTensorMutableData<float>();
                float* c_out_data = c_out_tensor.GetTensorMutableData<float>();

                // 按正确维度更新状态
                for (int layer = 0; layer < cfg.control.lstm_num_layers; ++layer) {
                    for (int hid = 0; hid < cfg.control.lstm_hidden_size; ++hid) {
                        int idx = layer * cfg.control.lstm_hidden_size + hid;
                        h_in_(layer, hid) = h_out_data[idx];
                        c_in_(layer, hid) = c_out_data[idx];
                    }
                }
            } else {
                std::cerr << "ONNX: LSTM output dimension mismatch!" << std::endl;
                std::cerr << "Expected h_out shape: [1, " << cfg.control.lstm_num_layers << ", " << cfg.control.lstm_hidden_size << "]" << std::endl;
                std::cerr << "Actual h_out shape: [";
                for (size_t i = 0; i < h_out_shape.size(); ++i) {
                    std::cerr << h_out_shape[i];
                    if (i < h_out_shape.size() - 1) std::cerr << ", ";
                }
                std::cerr << "]" << std::endl;
                
                std::cerr << "Actual c_out shape: [";
                for (size_t i = 0; i < c_out_shape.size(); ++i) {
                    std::cerr << c_out_shape[i];
                    if (i < c_out_shape.size() - 1) std::cerr << ", ";
                }
                std::cerr << "]" << std::endl;
                
                // 重置LSTM状态以防止后续错误
                h_in_.setZero();
                c_in_.setZero();
            }
        }
    }
    catch (const Ort::Exception &e)
    {
        std::cerr << "ONNX Runtime error during Run(): " << e.what() << std::endl;
        return;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Runtime error during ONNX inference: " << e.what() << std::endl;
        return;
    }
    catch (...)
    {
        std::cerr << "Unknown error during ONNX inference" << std::endl;
        return;
    }
    
    std::cerr << "ONNX: Inference completed, processing outputs" << std::endl;

    // 验证输出
    if (output_tensors.empty())
    {
        std::cerr << "ONNX Runtime returned no outputs." << std::endl;
        return;
    }

    auto &out_tensor = output_tensors[0];
    auto out_info = out_tensor.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> out_shape = out_info.GetShape();

    // 计算输出元素总数（对负维度/不定维度保守处理）
    size_t out_size = 1;
    for (auto d : out_shape)
    {
        if (d <= 0)
        {
            out_size = 0;
            break;
        }
        out_size *= static_cast<size_t>(d);
    }

    if (out_size < static_cast<size_t>(cfg.env.num_joints))
    {
        std::cerr << "ONNX output size " << out_size << " smaller than expected " << cfg.env.num_joints << std::endl;
        return;
    }

    // 处理输出并安全转换为 double
    float *output_data = out_tensor.GetTensorMutableData<float>();
    
    // [修改] 使用配置中的每关节限幅参数
    for (int i = 0; i < cfg.env.num_joints; ++i) {
        double raw_action = static_cast<double>(output_data[i]);
        
        // 获取关节特定的限幅值
        double current_min = -2.0; // 默认值
        double current_max = 2.0;  // 默认值
        
        if (i < cfg.control.action_clip_min.size()) {
            current_min = cfg.control.action_clip_min[i];
        }
        
        if (i < cfg.control.action_clip_max.size()) {
            current_max = cfg.control.action_clip_max[i];
        }
        
        actions_[i] = std::clamp(raw_action, current_min, current_max);
        
        // 可选：仅在限幅特别大时才警告，或者降低日志频率
        /*
        if (std::abs(raw_action) > std::max(std::abs(current_min), std::abs(current_max))) {
            std::cerr << "WARNING: Action[" << i << "] clamped from " 
                      << raw_action << " to [" << current_min << ", " << current_max << "]" << std::endl;
        }
        */
    }
}

Eigen::VectorXd AMPController::pd_output(const Eigen::VectorXd &target_q,
                                         const Eigen::VectorXd &q,
                                         const Eigen::VectorXd &target_dq,
                                         const Eigen::VectorXd &dq)
{
    // 确保输入向量的大小相同
    std::size_t n = target_q.size();
    Eigen::VectorXd torque(n);

    for (std::size_t i = 0; i < n; i++)
    {
        torque[i] = (target_q[i] - q[i]) * cfg.joint_params_isaaclab[i].kp +
                    (target_dq[i] - dq[i]) * cfg.joint_params_isaaclab[i].kd;
    }

    return torque;
}
