#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <algorithm> // for std::clamp
#include <cmath>
#include <deque>
#include <iostream>
#include <mujoco/mujoco.h>
#include <ros/package.h>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <chrono>
#include <xmlrpcpp/XmlRpcValue.h> // 添加这行

#include "ymbot_amp_devel/amp_controller.hpp" // 控制器
#include "ymbot_amp_devel/phase_generator.hpp" // PhaseGenerator

using namespace std;

class MujocoSimulation
{
public:
    mjModel *m = nullptr; // MuJoCo模型
    mjData *d = nullptr;  // MuJoCo数据
    mjvCamera cam;        // 相机
    mjvOption opt;        // 可视化选项
    mjvScene scn;         // 场景
    mjrContext con;       // 自定义GPU上下文

    bool button_left = false;   // 左键状态
    bool button_middle = false; // 中键状态
    bool button_right = false;  // 右键状态
    double lastx = 0;           // 上一次X坐标
    double lasty = 0;           // 上一次Y坐标

    // 添加 velocity_commands 成员变量gsj
    Eigen::VectorXd velocity_commands = Eigen::VectorXd::Zero(3);

    MujocoSimulation()
    {
        // 初始化MuJoCo模型和数据
        if (!glfwInit())
        {
            mju_error("Could not initialize GLFW");
        }
    }

    ~MujocoSimulation()
    {
        // 释放所有的MuJoCo资源
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        mj_deleteData(d);
        mj_deleteModel(m);
    }

    // 加载模型
    bool loadModel(const char *filename)
    {
        char error[1000] = "Could not load binary model";
        if (std::strlen(filename) > 4 && !std::strcmp(filename + std::strlen(filename) - 4, ".mjb"))
        {
            m = mj_loadModel(filename, 0); // 加载二进制模型
        }
        else
        {
            m = mj_loadXML(filename, 0, error, 1000); // 加载XML模型
        }
        if (!m)
        {
            mju_error("Load model error: %s", error);
            return false;
        }
        d = mj_makeData(m); // 创建模拟数据
        return true;
    }

    // // 键盘回调
    // static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    //     auto* sim = reinterpret_cast<MujocoSimulation*>(glfwGetWindowUserPointer(window));
    //     if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    //         mj_resetData(sim->m, sim->d); // 重置模拟数据
    //         mj_forward(sim->m, sim->d);   // 更新模型数据
    //     }
    // }

    // 键盘回调gsj
    static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
    {
        auto *sim = reinterpret_cast<MujocoSimulation *>(glfwGetWindowUserPointer(window));
        if (act == GLFW_PRESS)
        {
            switch (key)
            {
            case GLFW_KEY_BACKSPACE:
                mj_resetData(sim->m, sim->d); // 重置模拟数据
                mj_forward(sim->m, sim->d);   // 更新模型数据
                break;

            // 添加控制 velocity_commands 的按键
            case GLFW_KEY_W: // 增加 X 方向速度
                sim->velocity_commands[0] += 0.1;
                std::cout << "W pressed, velocity_commands[0] = " << sim->velocity_commands[0] << std::endl;
                break;
            case GLFW_KEY_S: // 减少 X 方向速度
                sim->velocity_commands[0] -= 0.1;
                std::cout << "S pressed, velocity_commands[0] = " << sim->velocity_commands[0] << std::endl;
                break;
            case GLFW_KEY_A: // 增加 Y 方向速度
                sim->velocity_commands[1] += 0.1;
                std::cout << "A pressed, velocity_commands[1] = " << sim->velocity_commands[1] << std::endl;
                break;
            case GLFW_KEY_D: // 减少 Y 方向速度
                sim->velocity_commands[1] -= 0.1;
                std::cout << "D pressed, velocity_commands[1] = " << sim->velocity_commands[1] << std::endl;
                break;
            case GLFW_KEY_Q: // 增加 Z 方向速度
                sim->velocity_commands[2] += 0.1;
                std::cout << "Q pressed, velocity_commands[2] = " << sim->velocity_commands[2] << std::endl;
                break;
            case GLFW_KEY_E: // 减少 Z 方向速度
                sim->velocity_commands[2] -= 0.1;
                std::cout << "E pressed, velocity_commands[2] = " << sim->velocity_commands[2] << std::endl;
                break;

            default:
                break;
            }
        }
    }
    // 鼠标按钮回调
    static void mouse_button(GLFWwindow *window, int button, int act, int mods)
    {
        auto *sim = reinterpret_cast<MujocoSimulation *>(glfwGetWindowUserPointer(window));
        sim->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        sim->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        sim->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        glfwGetCursorPos(window, &sim->lastx, &sim->lasty); // 获取当前鼠标位置
    }

    // 鼠标移动回调
    static void mouse_move(GLFWwindow *window, double xpos, double ypos)
    {
        auto *sim = reinterpret_cast<MujocoSimulation *>(glfwGetWindowUserPointer(window));

        if (!sim->button_left && !sim->button_middle && !sim->button_right)
        {
            return;
        }

        double dx = xpos - sim->lastx; // 计算鼠标X轴位移
        double dy = ypos - sim->lasty; // 计算鼠标Y轴位移
        sim->lastx = xpos;
        sim->lasty = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height); // 获取窗口尺寸

        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        mjtMouse action;
        if (sim->button_right)
        {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        }
        else if (sim->button_left)
        {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
        else
        {
            action = mjMOUSE_ZOOM;
        }

        mjv_moveCamera(sim->m, action, dx / height, dy / height, &sim->scn, &sim->cam); // 移动相机
    }

    // 滚轮回调
    static void scroll(GLFWwindow *window, double xoffset, double yoffset)
    {
        auto *sim = reinterpret_cast<MujocoSimulation *>(glfwGetWindowUserPointer(window));
        mjv_moveCamera(sim->m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &sim->scn, &sim->cam); // 缩放相机视角
    }
};

struct SimCfg
{
    std::string mujoco_model_path = "";
    double sim_duration = 120.0;
    double dt = 0.001;   // 0.001
    int decimation = 10; // 5
    SimCfg(const std::string &model_path) : mujoco_model_path(model_path) {}
};

// gsj
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> get_observation(const mjData *data,
                                                                                                                                 const mjModel *model)
{
    Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(data->qpos, model->nq);
    Eigen::VectorXd dq = Eigen::Map<Eigen::VectorXd>(data->qvel, model->nv);

    // 添加滤波相关变量
    static std::deque<Eigen::VectorXd> dq_history; // 存储历史dq值
    static const int filter_window = 5;            // 滤波窗口大小
    const double alpha = 0.2;                      // 低通滤波系数

    // 低通滤波处理dq (二选一)
    static Eigen::VectorXd filtered_dq = dq;
    filtered_dq = alpha * dq + (1 - alpha) * filtered_dq;
    dq = filtered_dq;

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
    // 检查sensor对应ID
    // for (int i = 0; i < model->nsensor; ++i)
    // {
    //     const char *name = mj_id2name(model, mjOBJ_SENSOR, i);
    //     printf("Sensor[%d]: %s (type=%d)\n", i, name ? name : "NULL", model->sensor_type[i]);
    // }
    // 获取四元数，mujoco中默认四元数顺序为 [w, x, y, z] 格式
    Eigen::VectorXd quat(4);
    int orientation_id = mj_name2id(model, mjOBJ_SENSOR, "orientation");
    int quat_start_idx = model->sensor_adr[orientation_id]; // 获取四元数在 sensordata 中的起始索引
    for (int i = 0; i < 4; ++i)
    {
        quat[i] = data->sensordata[quat_start_idx + i];
    }
    // 获取线速度，格式为 [vx, vy, vz]，世界坐标系

    static Eigen::VectorXd linvel(3);
    int linvel_id = mj_name2id(model, mjOBJ_SENSOR, "linear-velocity");
    // printf("linvel_id: %d      ", linvel_id);
    int linvel_start_idx = model->sensor_adr[linvel_id];
    // printf("linvel_start_idx: %d      ", linvel_start_idx);
    for (int i = 0; i < 3; ++i)
    {
        linvel[i] = data->sensordata[linvel_start_idx + i];
        // printf("linvel: %f", linvel[i]);
    }

    // 获取角速度，格式为 [wx, wy, wz]，世界坐标系
    static Eigen::VectorXd angvel(3);
    int angvel_id = mj_name2id(model, mjOBJ_SENSOR, "angular-velocity");
    int angvel_start_idx = model->sensor_adr[angvel_id];
    for (int i = 0; i < 3; ++i)
    {
        angvel[i] = data->sensordata[angvel_start_idx + i];
        // printf("angvel: %f", angvel[i]);
    }
    // printf("  \n");
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

    // // 获取基座的线速度和角速度
    // Eigen::VectorXd base_linear_velocity(3);
    // Eigen::VectorXd base_angular_velocity(3);

    // // 获取基座体的编号
    // int base_body_id = mj_name2id(model, mjOBJ_BODY, "base_link"); // 假设基座体的名字是 "base_link"

    // if (base_body_id >= 0) {
    //     for (int i = 0; i < 3; ++i) {
    //         base_angular_velocity[i] = data->cvel[base_body_id * 6 + i]; // 线速度
    //         base_linear_velocity[i] = data->cvel[base_body_id * 6 + 3 + i]; // 角速度
    //     }
    // } else {
    //     std::cerr << "Error: Base body 'base_link' not found in the model." << std::endl;
    // }

    return std::make_tuple(q, dq, quat, projected_gravity, linvel, angvel);
}

void run_mujoco(const SimCfg &sim_cfg, AMPController &amp_controller)
{
    // 创建 mujoco 仿真对象，包含大部分仿真内容
    MujocoSimulation mj_sim;
    if (!mj_sim.loadModel(sim_cfg.mujoco_model_path.c_str()))
    {
        return;
    }

    GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    // GLFWwindow* window = glfwCreateWindow(2560, 1440, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // 开启垂直同步

    mjv_defaultCamera(&mj_sim.cam);                          // 初始化相机
    mjv_defaultOption(&mj_sim.opt);                          // 初始化可视化选项
    mjv_defaultScene(&mj_sim.scn);                           // 初始化场景
    mjr_defaultContext(&mj_sim.con);                         // 初始化GPU上下文
    mjv_makeScene(mj_sim.m, &mj_sim.scn, 2000);              // 创建场景
    mjr_makeContext(mj_sim.m, &mj_sim.con, mjFONTSCALE_150); // 创建渲染上下文

    glfwSetWindowUserPointer(window, &mj_sim);               // 设置窗口的用户指针为当前对象
    glfwSetKeyCallback(window, mj_sim.keyboard);             // 设置键盘回调
    glfwSetCursorPosCallback(window, mj_sim.mouse_move);     // 设置鼠标移动回调
    glfwSetMouseButtonCallback(window, mj_sim.mouse_button); // 设置鼠标按钮回调
    glfwSetScrollCallback(window, mj_sim.scroll);            // 设置滚轮回调

    // 初始化仿真数据
    mj_sim.m->opt.timestep = sim_cfg.dt; // 设置仿真步长

    // 设置机器人关节的初始值
    for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx)
    {
        size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
        // mj_sim.d->qpos[mj_sim.m->nq - amp_controller.cfg.env.num_joints + mjc_idx] =
        //     amp_controller.cfg.joint_params_isaaclab[lab_idx].offset;

        mj_sim.d->qpos[mj_sim.m->nq - amp_controller.cfg.env.num_joints + mjc_idx] = 0.0;
    }
    std::cerr << "model_nq = :" << mj_sim.m->nq << std::endl;
    mj_step(mj_sim.m, mj_sim.d);

    // std::cout << "All joint names:" << std::endl;
    // for (int i = 0; i < mj_sim.m->njnt; ++i) {
    //     const char* joint_name = mj_id2name(mj_sim.m, mjOBJ_JOINT, i);
    //     if (joint_name) {
    //         std::cout << "\"" << joint_name << "\"," << std::endl;
    //     }
    // }
    // cout << "mj_sim.m->nq: " << mj_sim.m->nq << endl;
    // return;

    int count_pd = 0;
    int count_onnx = 0;
    double time_pd = 1.0; // s
    Eigen::VectorXd first_q = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);

    // cmd
    Eigen::VectorXd velocity_commands = Eigen::VectorXd::Zero(3);
    velocity_commands[0] = 0.0;
    velocity_commands[1] = 0.0;
    velocity_commands[2] = 0.0;

    Eigen::VectorXd q_lab = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd dq_lab = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd tau_lab(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd tau_mjc(amp_controller.cfg.env.num_joints);

    Eigen::VectorXd target_q = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);
    Eigen::VectorXd target_dq = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);

    Eigen::VectorXd target_q_record = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints);

    // 定义 ONNX 需要的变量
    Eigen::VectorXd action = Eigen::VectorXd::Zero(amp_controller.cfg.env.num_joints); 
    Eigen::VectorXd phase = Eigen::VectorXd::Zero(6);
    
    // 创建 PhaseGenerator 对象，与 flyrobot-wjc-mujoco-sim 保持一致
    PhaseGenerator phase_generator;

    // Eigen::VectorXd pd_target_test(amp_controller.cfg.env.num_joints);
    // pd_target_test << -0.1, 0.0, 0.0, 0.3, -0.20, 0.0,
    //     -0.1, 0.0, 0.0, 0.3, -0.20, 0.0,
    //     0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0;
    // 调试：打开文件用于写入数据
    std::ofstream outfile("/home/pc/ymzzgaitdeploymentcv2250916/src/data/sim2sim_onnx_rec_joint.txt", std::ios::out);
    std::ofstream onnx_rec("/home/pc/ymzzgaitdeploymentcv2250916/src/data/sim2real_onnx_rec_joint.txt", std::ios::out);
    std::ofstream onnx_send("/home/pc/ymzzgaitdeploymentcv2250916/src/data/sim2real_onnx_send_joint.txt", std::ios::out);
    std::ofstream sensor_data("/home/pc/ymzzgaitdeploymentcv2250916/src/data/sim2real_sensor.txt", std::ios::out);
    /*可以手动调整仿真的快慢, 与屏幕刷新率有关
     * 例如屏幕刷新率为60hz, 则一帧时间为16.7ms
     * mojuco 仿真引擎时间单位 mj_sim.m->opt.timestep=1ms,所以只要令 steps=16, 即可让机器人以正常的速度运动
     * 其他刷新率, 例如144hz对应6.9ms, 则需要令 steps=6
     * */
    // 设置速度因子, 更加方便
    double speed_factor = 1;
    double screen_refresh_rate = 60.0;
    double steps = 1.6 / screen_refresh_rate / mj_sim.m->opt.timestep * speed_factor;
    while (!glfwWindowShouldClose(window))
    {

        for (int n = 0; n < steps; n++)
        {
            // 获取机器人状态
            // gsj
            auto [q_mjc, dq_mjc, quat, projected_gravity, linvel_mjc, angvel_mjc] = get_observation(mj_sim.d, mj_sim.m);

            // std::cout << "Base Linear Velocity: [" << base_linear_velocity[0] << ", "
            //           << base_linear_velocity[1] << ", " << base_linear_velocity[2] << "]" << std::endl;
            // std::cout << "Base Angular Velocity: [" << base_angular_velocity[0] << ", "
            //           << base_angular_velocity[1] << ", " << base_angular_velocity[2] << "]" << std::endl;
            if (sensor_data.is_open())
            {
                sensor_data << count_onnx * sim_cfg.dt;
                for (int i = 0; i < projected_gravity.size(); ++i)
                {
                    sensor_data << "," << projected_gravity[i];
                }
                for (int i = 0; i < linvel_mjc.size(); ++i)
                {
                    sensor_data << "," << linvel_mjc[i];
                }
                for (int i = 0; i < angvel_mjc.size(); ++i)
                {
                    sensor_data << "," << angvel_mjc[i];
                }
                sensor_data << "\n";
            }
            // gsj
            // 打印当前速度指令
            // std::cout << "[x: " << mj_sim.velocity_commands[0] << ", y: "
            //           << mj_sim.velocity_commands[1] << ", yaw: " << mj_sim.velocity_commands[2] << "]" << std::endl;

            // 使用实时更新的 velocity_commands

            velocity_commands = mj_sim.velocity_commands;

            q_mjc = q_mjc.tail(amp_controller.cfg.env.num_joints);   // 提取与 acitons 对应的关节位置
            dq_mjc = dq_mjc.tail(amp_controller.cfg.env.num_joints); // 提取与 acitons 对应的关节速度
            // for (size_t i = 0; i < 8; i++)
            // {
            //     // q_mjc[i + 14] = 0;
            //     dq_mjc[i + 14] = 0;
            //     std::cout << "Base Linear Velocity: [" << i << "]  : " << dq_mjc[i + 14] << " ." << std::endl;
            // }

            // 对 q 和 dq 重新排序，调整方向
            for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx)
            {
                size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
                // printf("lab_idx : %d", lab_idx);
                q_lab[lab_idx] = q_mjc[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                dq_lab[lab_idx] = dq_mjc[mjc_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
            }
            if (count_pd < time_pd / mj_sim.m->opt.timestep)
            { // PD 站立模式
                if (count_pd < 1)
                {
                    first_q = q_mjc;
                }
                count_pd++;

                // 定义一组额外的kp 和 kd，用于站立
                Eigen::VectorXd kps(amp_controller.cfg.env.num_joints);
                kps.setConstant(300); // 一次性设置所有元素为300

                Eigen::VectorXd kds(amp_controller.cfg.env.num_joints);
                kds.setConstant(1); // 同样方法设置kds

                for (int i = 0; i < amp_controller.cfg.env.num_joints; i++)
                {
                    // target_q[i] = first_q[i] + count_pd * mj_sim.m->opt.timestep / time_pd * (0 - first_q[i]);
                    target_q[i] = first_q[i] + count_pd * mj_sim.m->opt.timestep / time_pd *
                                                   (amp_controller.cfg.joint_params_isaaclab[i].offset - first_q[i]);
                }
                // for (int i = 0; i < 13; i++)
                // {
                //     std::cerr << "q_lab : [" << i << "]    " << target_q[i] << "      dq_lab : " << target_q[i] << std::endl;
                // }
                // 生成力矩信号
                for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                {
                    tau_lab[i] = (target_q[i] - q_lab[i]) * kps[i] + (target_dq[i] - dq_lab[i]) * kds[i];
                    tau_lab[i] = std::clamp(tau_lab[i], -amp_controller.cfg.joint_params_isaaclab[i].tau_limit,
                                            amp_controller.cfg.joint_params_isaaclab[i].tau_limit);
                    tau_lab[i] *= amp_controller.cfg.joint_params_isaaclab[i].direction;
                }

                // 重新排序 tau 到 mujoco 中的关节顺序
                for (size_t i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                {
                    tau_mjc[i] = tau_lab[amp_controller.cfg.mjc2lab[i]];
                    // printf("tau_mjc : %d", amp_controller.cfg.mjc2lab[i]);
                }

                for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                {
                    mj_sim.d->ctrl[i] = tau_mjc[i];
                    // mj_sim.d->ctrl[i] = 0.0;
                }

                cout << "count_pd: " << count_pd << endl;
            }
            else
            { // 强化学习走路模式
                count_onnx++;
                // for (size_t i = 0; i < amp_controller.cfg.env.num_joints; i++)
                // {
                //     q_mjc[i] = pd_target_test[i];
                // }
                // 调用rl控制器,1000hz->100hz
                if (count_onnx % sim_cfg.decimation == 0)
                {
                    // auto start_time = std::chrono::steady_clock::now();
                    // gsj
                    
                    // [新增] 使用 PhaseGenerator 生成 phase 观测
                    Eigen::Vector3d cmd_vel_vector(velocity_commands[0], velocity_commands[1], velocity_commands[2]);
                    phase = phase_generator.generatePhase(linvel_mjc, cmd_vel_vector, sim_cfg.dt);
                    
                    amp_controller.onnx_output(linvel_mjc, angvel_mjc, projected_gravity, velocity_commands, q_lab, dq_lab, action, phase);
                    //  放缩和补偿
                    for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                    {
                        target_q[i] = amp_controller.actions_[i] * amp_controller.cfg.control.actions_;
                        target_q[i] += amp_controller.cfg.joint_params_isaaclab[i].offset;
                        target_q[i] = std::clamp(target_q[i], -amp_controller.cfg.control.clip_actions,
                                                 amp_controller.cfg.control.clip_actions);
                        // amp_controller.actions_[i] = target_q[i];
                    }

                    for (size_t i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                    {
                        size_t lab_idx = amp_controller.cfg.mjc2lab[i];
                        target_q_record[i] = target_q[lab_idx] * amp_controller.cfg.joint_params_isaaclab[lab_idx].direction;
                    }
                    // cout << "action: " << amp_controller.actions_ << endl;

                    // // 监测
                    // auto end_time = std::chrono::steady_clock::now();tau
                    // auto elapsed_microseconds =
                    //     std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                    // std::cout << "Elapsed time: " << elapsed_microseconds.count() << "us\n";
                }
                if (onnx_send.is_open())
                {
                    onnx_send << count_onnx * sim_cfg.dt;
                    for (int i = 0; i < target_q_record.size(); ++i)
                    {
                        onnx_send << "," << target_q_record[i];
                    }
                    onnx_send << "\n";
                }
                // 生成力矩信号
                Eigen::VectorXd tau_lab = amp_controller.pd_output(target_q, q_lab, target_dq, dq_lab);

                // 限幅和调整方向
                for (size_t i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                {
                    tau_lab[i] = std::clamp(tau_lab[i], -amp_controller.cfg.joint_params_isaaclab[i].tau_limit,
                                            amp_controller.cfg.joint_params_isaaclab[i].tau_limit);
                    tau_lab[i] *= amp_controller.cfg.joint_params_isaaclab[i].direction;
                }

                // 重新排序 tau 到 mujoco 中的关节顺序
                for (size_t mjc_idx = 0; mjc_idx < amp_controller.cfg.env.num_joints; ++mjc_idx)
                {
                    size_t lab_idx = amp_controller.cfg.mjc2lab[mjc_idx];
                    tau_mjc[mjc_idx] = tau_lab[lab_idx];
                }

                for (int i = 0; i < amp_controller.cfg.env.num_joints; ++i)
                {
                    // mj_sim.d->ctrl[i] = tau[i];
                    mj_sim.d->ctrl[i] = tau_mjc[i];
                    // 调试
                    //  mj_sim.d->ctrl[i] = 0.0;
                }
                if (onnx_rec.is_open())
                {
                    onnx_rec << count_onnx * sim_cfg.dt;
                    for (int i = 0; i < q_mjc.size(); ++i)
                    {
                        onnx_rec << "," << q_mjc[i];
                    }
                    onnx_rec << "\n";
                }
                // for (int i = 0; i < q_mjc.size(); i++)
                // {
                //     std::cerr << "UP: q_mjc :" << i << "    " << q_mjc[i] << std::endl;
                // }
                // if (onnx_send.is_open())
                // {
                //     onnx_send << count_onnx * sim_cfg.dt;
                //     for (int i = 0; i < target_q_record.size(); ++i)
                //     {
                //         onnx_send << "," << target_q_record[i];
                //     }
                //     onnx_send << "\n";
                // }
                // cout << endl;

                // // 将 想要查看的数据写入文件，每个周期一行, 减少数据量，1000hz->100hz
                // if (count_onnx % sim_cfg.decimation == 0) {
                //     if (outfile.is_open()) {
                //         outfile << count_onnx * sim_cfg.dt;
                //         for (int i = 0; i < q.size(); ++i) {
                //             outfile << "," << q[i];
                //         }
                //         for (int i = 0; i < dq.size(); ++i) {
                //             outfile << "," << dq[i];
                //         }
                //         for (int i = 0; i < target_q.size(); ++i) {
                //             outfile << "," << target_q[i] + amp_controller.default_dof_pos_[i];
                //         }
                //         // for (int i = 0; i < eu_ang.size(); ++i) {
                //         //     outfile << "," <<eu_ang[i];
                //         // }
                //         outfile << "\n";
                //     }
                // }

                // cout << "count_onnx: " << count_onnx << endl;
            }

            // 进行仿真步进
            mj_step(mj_sim.m, mj_sim.d);

            // 调试和导出数据
            // for (int i = 0; i < cfg.env.num_joints; ++i) {
            //     // cout << mj_sim.d->ctrl[i] << " ";
            //     cout << tau[i] << " ";
            // }
            // cout << endl;
        }
        // 渲染仿真
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height); // 获取帧缓冲尺寸

        // 更新场景
        mjv_updateScene(mj_sim.m, mj_sim.d, &mj_sim.opt, nullptr, &mj_sim.cam, mjCAT_ALL, &mj_sim.scn);

        // 渲染场景
        mjr_render(viewport, &mj_sim.scn, &mj_sim.con);

        // 交换缓冲区并处理事件
        glfwSwapBuffers(window);
        glfwPollEvents();
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

    SimCfg sim_cfg(model_path);
    AMPController amp_controller(onnx_path, num_joints, joint_params_isaaclab, joint_names_mujoco);
    run_mujoco(sim_cfg, amp_controller);

    return 0;
}
