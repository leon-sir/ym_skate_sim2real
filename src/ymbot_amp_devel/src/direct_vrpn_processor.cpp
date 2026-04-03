#include "ymbot_amp_devel/direct_vrpn_processor.hpp"

#include <cmath>

namespace ymbot_amp_devel
{

DirectVrpnProcessor::DirectVrpnProcessor(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.param<std::string>("vrpn_topic", vrpn_topic_, "/vrpn_client_node/Rigid/pose");
    nh_.param<bool>("direct_vrpn_debug", debug_output_, false);
    nh_.param<double>("direct_vrpn_lpf_alpha_omega", lpf_alpha_omega_, 0.15);
    nh_.param<double>("direct_vrpn_lpf_alpha_vel", lpf_alpha_vel_, 0.2);
    nh_.param<double>("direct_vrpn_dt", processing_dt_, 0.02);

    if (processing_dt_ <= 0.0)
    {
        ROS_WARN("direct_vrpn_dt <= 0. Falling back to 0.02s.");
        processing_dt_ = 0.02;
    }

    vrpn_sub_ = nh_.subscribe(vrpn_topic_, 10, &DirectVrpnProcessor::poseCallback, this);
    timer_ = nh_.createTimer(ros::Duration(processing_dt_), &DirectVrpnProcessor::timerCallback, this);

    ROS_INFO("Direct VRPN processor started. Input topic: %s, dt: %.3f s",
             vrpn_topic_.c_str(), processing_dt_);
}

DirectVrpnState DirectVrpnProcessor::getState() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return state_;
}

void DirectVrpnProcessor::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    curr_pos_zup_ = Eigen::Vector3d(
        msg->pose.position.x,
        -msg->pose.position.z,
        msg->pose.position.y);

    Eigen::Quaterniond q_raw(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);

    if (q_raw.norm() < 1e-8)
    {
        ROS_WARN_THROTTLE(1.0, "Received near-zero VRPN quaternion. Ignoring this frame.");
        return;
    }

    q_raw.normalize();
    q_raw_ = q_raw;
    q_zup_ = Eigen::Quaterniond(q_raw.w(), q_raw.x(), -q_raw.z(), q_raw.y());
    q_zup_.normalize();

    header_ = msg->header;
    has_data_ = true;
}

void DirectVrpnProcessor::timerCallback(const ros::TimerEvent &event)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!has_data_)
    {
        return;
    }

    const Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
    state_.projected_gravity = q_zup_.inverse() * gravity_world;

    if (has_last_state_)
    {
        const Eigen::Vector3d v_world = (curr_pos_zup_ - last_pos_zup_) / processing_dt_;
        const Eigen::Vector3d v_body_raw = q_zup_.inverse() * v_world;
        last_vel_body_ = lpf_alpha_vel_ * v_body_raw + (1.0 - lpf_alpha_vel_) * last_vel_body_;
        state_.base_linear_velocity = last_vel_body_;

        Eigen::Quaterniond last_q_aligned = last_q_zup_;
        if (q_zup_.coeffs().dot(last_q_aligned.coeffs()) < 0.0)
        {
            last_q_aligned.coeffs() = -last_q_aligned.coeffs();
        }

        Eigen::Quaterniond dq;
        dq.coeffs() = (q_zup_.coeffs() - last_q_aligned.coeffs()) / processing_dt_;

        const Eigen::Quaterniond omega_quat = q_zup_.inverse() * dq;
        const Eigen::Vector3d omega_body_raw(
            2.0 * omega_quat.x(),
            2.0 * omega_quat.y(),
            2.0 * omega_quat.z());

        last_omega_body_ =
            lpf_alpha_omega_ * omega_body_raw + (1.0 - lpf_alpha_omega_) * last_omega_body_;
        state_.base_angular_velocity = last_omega_body_;
    }
    else
    {
        state_.base_linear_velocity.setZero();
        state_.base_angular_velocity.setZero();
    }

    const Eigen::Vector3d forward_world = q_raw_.normalized() * Eigen::Vector3d::UnitX();
    state_.heading = std::atan2(forward_world.y(), forward_world.x());
    state_.stamp = header_.stamp.isZero() ? event.current_real : header_.stamp;
    state_.has_pose = true;
    state_.has_velocity = has_last_state_;

    if (debug_output_ && has_last_state_)
    {
        ROS_INFO_THROTTLE(
            1.0,
            "[Direct VRPN] Grav: [%.4f, %.4f, %.4f] | Vel: [%.4f, %.4f, %.4f] | Omega: [%.4f, %.4f, %.4f]",
            state_.projected_gravity.x(), state_.projected_gravity.y(), state_.projected_gravity.z(),
            state_.base_linear_velocity.x(), state_.base_linear_velocity.y(), state_.base_linear_velocity.z(),
            state_.base_angular_velocity.x(), state_.base_angular_velocity.y(), state_.base_angular_velocity.z());
    }

    last_pos_zup_ = curr_pos_zup_;
    last_q_zup_ = q_zup_;
    has_last_state_ = true;
}

} // namespace ymbot_amp_devel
