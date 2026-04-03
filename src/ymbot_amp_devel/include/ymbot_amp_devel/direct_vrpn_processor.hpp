#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <string>

namespace ymbot_amp_devel
{

struct DirectVrpnState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d projected_gravity = Eigen::Vector3d(0.0, 0.0, -1.0);
    Eigen::Vector3d base_linear_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_angular_velocity = Eigen::Vector3d::Zero();
    double heading = 0.0;
    ros::Time stamp;
    bool has_pose = false;
    bool has_velocity = false;
};

class DirectVrpnProcessor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit DirectVrpnProcessor(ros::NodeHandle &nh);

    DirectVrpnState getState() const;

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &event);

    ros::NodeHandle nh_;
    ros::Subscriber vrpn_sub_;
    ros::Timer timer_;

    mutable std::mutex data_mutex_;

    std::string vrpn_topic_;
    bool debug_output_ = false;
    double lpf_alpha_omega_ = 0.15;
    double lpf_alpha_vel_ = 0.2;
    double processing_dt_ = 0.02;

    std_msgs::Header header_;
    bool has_data_ = false;
    bool has_last_state_ = false;

    DirectVrpnState state_;

    Eigen::Vector3d curr_pos_zup_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_pos_zup_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_omega_body_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_vel_body_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_zup_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond last_q_zup_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_raw_ = Eigen::Quaterniond::Identity();
};

} // namespace ymbot_amp_devel
