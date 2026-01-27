#ifndef PHASE_GENERATOR_HPP
#define PHASE_GENERATOR_HPP

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

/**
 * @brief 生成6维phase观测的工具类
 * 6维phase包括: [left_stance, left_skate, left_flight, right_stance, right_skate, right_flight]
 * 按照Python端skating_phase_v2函数输出格式：phase_obs(1,2,3)对应左腿，phase_obs(4,5,6)对应右腿
 */
class PhaseGenerator {
public:
    /**
     * @brief 构造函数
     * @param base_frequency 基础频率
     * @param base_duty1 基础占空比
     */
    PhaseGenerator(double base_frequency = 0.65, double base_duty1 = 0.5) 
        : base_frequency_(base_frequency), base_duty1_(base_duty1), 
          current_gait_index_(0.0), duty1_left_(base_duty1), duty1_right_(base_duty1) {
        // 设置参数范围
        min_duty_ = 0.4;
        max_duty_ = 0.95;
        min_frequency_ = 0.5;
        max_frequency_ = 1.0;
        max_tracking_error_ = 0.75;
        max_ang_cmd_ = 1.0;
        max_duty_diff_ = 0.3;
        duty_ramp_tau_ = 0.02;  // 0.02
        freq_ramp_tau_ = 0.02;  // 0.02
        step_dt_ = 0.02;  // 控制周期，默认20ms
        
        alpha_d_ = 1.0 - std::exp(-step_dt_ / duty_ramp_tau_);
        alpha_f_ = 1.0 - std::exp(-step_dt_ / freq_ramp_tau_);
        
        // 初始化相位偏移
        phase_offset_[0] = 0.5;  // 左腿相位偏移
        phase_offset_[1] = 0.0;  // 右腿相位偏移
    }

    /**
     * @brief 根据当前状态生成6维phase观测
     * @param base_lin_vel 机器人基座线速度
     * @param cmd 速度命令 [vx, vy, wz]
     * @param dt 时间步长
     * @return 6维phase观测向量
     * 其中 phase_obs（1，2，3）左腿，phase_obs（4，5，6）右腿
     */
    Eigen::VectorXd generatePhase(const Eigen::Vector3d& base_lin_vel, 
                                const Eigen::Vector3d& cmd,
                                double dt = 0.02) {
        // 更新时间步长
        step_dt_ = dt;
        alpha_d_ = 1.0 - std::exp(-step_dt_ / duty_ramp_tau_);
        alpha_f_ = 1.0 - std::exp(-step_dt_ / freq_ramp_tau_);

        // 计算 x 方向的线速度跟踪误差（与训练代码一致）
        // x_tracking_error = cmd[0] - base_lin_vel[0]
        double x_tracking_error = cmd(0) - base_lin_vel(0);

        // 根据跟踪误差映射到期望的占空比和频率
        std::pair<double, double> gait_params = mapVxTrackingErrorToGait(x_tracking_error);
        double desired_duty = gait_params.first;
        double desired_freq = gait_params.second;

        // 根据角速度命令计算占空比差异
        double duty_diff = mapAngVelCmdToDutyDiff(cmd(2));  // cmd(2) 是 wz

        // 更新左右腿的占空比
        duty1_left_ = (1.0 - alpha_d_) * duty1_left_ + 
                     alpha_d_ * std::max(min_duty_, std::min(max_duty_, desired_duty + duty_diff));
        duty1_right_ = (1.0 - alpha_d_) * duty1_right_ + 
                      alpha_d_ * std::max(min_duty_, std::min(max_duty_, desired_duty - duty_diff));

        // 计算左右腿的第二阶段占空比
        double duty2_left = 0.5 + duty1_left_ / 2.0;
        double duty2_right = 0.5 + duty1_right_ / 2.0;

        // 每条腿三个相位的占空比，确保总和为1
        // 第一相：stance phase，从0到duty1
        // 第二相：skate phase，从duty1到duty2
        // 第三相：flight phase，从duty2到1.0
        double left_duty2 = duty2_left;
        double right_duty2 = duty2_right;

        // 限制duty2不能超过1，防止占空比总和超过1
        left_duty2 = std::min(left_duty2, 1.0);
        right_duty2 = std::min(right_duty2, 1.0);

        // 更新基础频率
        base_frequency_ = (1.0 - alpha_f_) * base_frequency_ + alpha_f_ * desired_freq;

        // 更新步态索引
        current_gait_index_ = std::fmod(current_gait_index_ + base_frequency_ * step_dt_, 1.0);

        // 计算左右腿相位
        double left_phase = std::fmod(phase_offset_[0] + current_gait_index_, 1.0);
        double right_phase = std::fmod(phase_offset_[1] + current_gait_index_, 1.0);

        // 生成6维phase观测: [left_stance, left_skate, left_flight, right_stance, right_skate, right_flight]
        // 使用与训练代码完全一致的平滑 CDF 方法
        Eigen::VectorXd phase_obs(6);
        
        const double kappa_gait_probs = 0.02;
        auto smoothing_cdf_start = [kappa_gait_probs](double x) {
            return 0.5 * (1.0 + std::erf(x / (kappa_gait_probs * std::sqrt(2.0))));
        };

        // 左腿三个相位的权重
        double f_left = left_phase;
        double d1_left = duty1_left_;
        double d2_left = left_duty2;
        
        // phase 1: stance (0 到 d1)
        phase_obs(0) = smoothing_cdf_start(f_left) * (1.0 - smoothing_cdf_start(f_left - d1_left))
                     + smoothing_cdf_start(f_left - 1.0) * (1.0 - smoothing_cdf_start(f_left - d1_left - 1.0));
        
        // phase 2: skate (d1 到 d2)
        phase_obs(1) = smoothing_cdf_start(f_left - d1_left) * (1.0 - smoothing_cdf_start(f_left - d2_left))
                     + smoothing_cdf_start(f_left - d1_left - 1.0) * (1.0 - smoothing_cdf_start(f_left - d2_left - 1.0));
        
        // phase 3: flight (d2 到 1.0)
        phase_obs(2) = smoothing_cdf_start(f_left - d2_left) * (1.0 - smoothing_cdf_start(f_left - 1.0))
                     + smoothing_cdf_start(f_left - d2_left - 1.0) * (1.0 - smoothing_cdf_start(f_left - 2.0));

        // 右腿三个相位的权重
        double f_right = right_phase;
        double d1_right = duty1_right_;
        double d2_right = right_duty2;
        
        // phase 1: stance (0 到 d1)
        phase_obs(3) = smoothing_cdf_start(f_right) * (1.0 - smoothing_cdf_start(f_right - d1_right))
                     + smoothing_cdf_start(f_right - 1.0) * (1.0 - smoothing_cdf_start(f_right - d1_right - 1.0));
        
        // phase 2: skate (d1 到 d2)
        phase_obs(4) = smoothing_cdf_start(f_right - d1_right) * (1.0 - smoothing_cdf_start(f_right - d2_right))
                     + smoothing_cdf_start(f_right - d1_right - 1.0) * (1.0 - smoothing_cdf_start(f_right - d2_right - 1.0));
        
        // phase 3: flight (d2 到 1.0)
        phase_obs(5) = smoothing_cdf_start(f_right - d2_right) * (1.0 - smoothing_cdf_start(f_right - 1.0))
                     + smoothing_cdf_start(f_right - d2_right - 1.0) * (1.0 - smoothing_cdf_start(f_right - 2.0));

        // 注意：不进行归一化！训练代码中没有归一化，CDF 公式在某些相位下和不等于 1.0 是正常的
        // 这是 CDF 平滑方法的数学特性，保持与训练环境完全一致

        return phase_obs;
    }

    /**
     * @brief 重置phase生成器
     */
    void reset() {
        current_gait_index_ = 0.0;
        duty1_left_ = base_duty1_;
        duty1_right_ = base_duty1_;
    }

    // Getter and Setter 方法
    double getBaseFrequency() const { return base_frequency_; }
    void setBaseFrequency(double frequency) { base_frequency_ = frequency; }

    double getBaseDuty1() const { return base_duty1_; }
    void setBaseDuty1(double duty1) { base_duty1_ = duty1; }

    double getCurrentGaitIndex() const { return current_gait_index_; }
    void setCurrentGaitIndex(double index) { current_gait_index_ = index; }
    
    double getDuty1Left() const { return duty1_left_; }
    double getDuty1Right() const { return duty1_right_; }

private:
    // 参数
    double base_frequency_;
    double base_duty1_;
    double current_gait_index_;
    double duty1_left_;
    double duty1_right_;

    // 参数范围
    double min_duty_;
    double max_duty_;
    double min_frequency_;
    double max_frequency_;
    double max_tracking_error_;
    double max_ang_cmd_;
    double max_duty_diff_;
    double duty_ramp_tau_;
    double freq_ramp_tau_;
    double step_dt_;
    double alpha_d_;
    double alpha_f_;
    double phase_offset_[2];

    /**
     * @brief 根据线速度跟踪误差映射到占空比和频率
     * @param x_tracking_error 线速度跟踪误差
     * @return std::pair<duty, frequency> 占空比和频率
     */
    std::pair<double, double> mapVxTrackingErrorToGait(double x_tracking_error) {
        double norm = std::max(0.0, std::min(1.0, x_tracking_error / max_tracking_error_));

        // 占空比：误差小时接近最大值，误差大时接近最小值
        double duty = min_duty_ + (1.0 - norm) * (max_duty_ - min_duty_);

        // 频率：误差小时接近最小值，误差大时接近最大值
        double freq = min_frequency_ + norm * (max_frequency_ - min_frequency_);

        return {duty, freq};
    }

    /**
     * @brief 根据角速度命令映射到占空比差异
     * @param ang_vel_cmd 角速度命令
     * @return 占空比差异
     */
    double mapAngVelCmdToDutyDiff(double ang_vel_cmd) {
        double norm = std::clamp(ang_vel_cmd / max_ang_cmd_, -1.0, 1.0);
        return norm * max_duty_diff_;
    }
};

#endif // PHASE_GENERATOR_HPP