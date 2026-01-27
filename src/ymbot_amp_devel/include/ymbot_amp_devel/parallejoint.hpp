#ifndef PARALLEJOINT_H
#define PARALLEJOINT_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>


class ParalleJoint {
  public:
    // 构造函数
    ParalleJoint(double lbar, double lrod1, double lrod2, double lspace, double az1, double az2, double cz);
    void frame(int i);

    // 计算绕y轴旋转的矩阵
    Eigen::Matrix3d RyPlot(double pitch);

    // 计算绕x轴和y轴旋转的矩阵
    Eigen::Matrix3d xrot(double pitch, double roll);

    // 给定末端目标俯仰角和滚转角，返回电机角度
    Eigen::Vector2d ik(double pitch, double roll);

    // 输入俯仰角横滚角电机角度1，2，计算当前雅可比矩阵
    Eigen::Matrix2d Jac(double pitch, double roll, double theta1, double theta2, Eigen::Matrix2d& JcInv);

    // 前向动力学，给定目标俯仰角和滚转角，计算电机角度
    Eigen::Vector2d fw(double pitchRef, double rollRef, double theta1, double theta2);

    // 计算力和速度
    // jointVel: 关节速度， motorForce: 末端受力， motorVel: 末端速度， jointForce: 关节力
    void forceVelJoint2Motor(double pitch,
                             double roll,
                             double theta1,
                             double theta2,
                             const Eigen::Vector2d& jointVel,
                             const Eigen::Vector2d& jointForce,
                             Eigen::Vector2d& motorVel,
                             Eigen::Vector2d& motorForce);
    void forceVelMotor2Joint(double pitch,
                             double roll,
                             double theta1,
                             double theta2,
                             Eigen::Vector2d& jointVel,
                             Eigen::Vector2d& jointForce,
                             const Eigen::Vector2d& motorVel,
                             const Eigen::Vector2d& motorForce);

  private:
    // 机器人参数
    double lbar, lrod1, lrod2, lspace;
    double ax, az1_, az2_, cz_;
    Eigen::Vector3d ra01, ra02, rc01, rc02, rab01, rab02;
};

#endif // PARALLELANKLE_H