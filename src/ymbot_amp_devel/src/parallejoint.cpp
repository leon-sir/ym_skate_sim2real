#include "ymbot_amp_devel/parallejoint.hpp"

ParalleJoint::ParalleJoint(double lbar, double lrod1, double lrod2, double lspace, double az1, double az2, double cz)
    : lbar(lbar), lrod1(lrod1), lrod2(lrod2), lspace(lspace), az1_(az1), az2_(az2), cz_(cz)
{
    ax = 0; // x轴向偏移
    // az = 0.136;
    // az2 = 0.07  ;

    // ra01 << ax , (lspace * 0.5), az ;    //电机转轴1坐标位置
    // ra02 << ax , (-lspace * 0.5), az2 ;   //电机转轴2坐标位置
    // rc01 << -lbar , (lspace * 0.5), 0;  //脚掌连杆转轴1位置
    // rc02 << -lbar , (-lspace * 0.5), 0;  //脚掌连杆转轴2位置
    // rab01 << -lbar, 0, 0;        //电机摆臂转轴1到电机转轴的偏移
    // rab02 << -lbar, 0, 0;        //电机摆臂转轴2到电机转轴的偏移
}
void ParalleJoint::frame(int i)
{
    if (i < 6)
    {
        ra01 << ax, (lspace * 0.5), az1_;    // 电机转轴1坐标位置
        ra02 << ax, (-lspace * 0.5), az2_;   // 电机转轴2坐标位置
        rc01 << -lbar, (lspace * 0.5), cz_;  // 脚掌连杆转轴1位置
        rc02 << -lbar, (-lspace * 0.5), cz_; // 脚掌连杆转轴2位置
        rab01 << -lbar, 0, 0;                // 电机摆臂转轴1到电机转轴的偏移
        rab02 << -lbar, 0, 0;                // 电机摆臂转轴2到电机转轴的偏移
    }
    else if (i >= 6 && i < 12)
    {
        ra01 << ax, (lspace * 0.5), az2_;    // 电机转轴1坐标位置
        ra02 << ax, (-lspace * 0.5), az1_;   // 电机转轴2坐标位置
        rc01 << -lbar, (lspace * 0.5), cz_;  // 脚掌连杆转轴1位置
        rc02 << -lbar, (-lspace * 0.5), cz_; // 脚掌连杆转轴2位置
        rab01 << -lbar, 0, 0;                // 电机摆臂转轴1到电机转轴的偏移
        rab02 << -lbar, 0, 0;                // 电机摆臂转轴2到电机转轴的偏移
    }
}

/*****绕y轴旋转的矩阵********/
Eigen::Matrix3d ParalleJoint::RyPlot(double pitch)
{
    Eigen::Matrix3d Ry;
    Ry << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
    return Ry;
}
/*********绕x轴和y轴旋转***************/
Eigen::Matrix3d ParalleJoint::xrot(double pitch, double roll)
{
    Eigen::Matrix3d Ry;
    Ry << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

    return Ry * Rx;
}
/*******给定末端的目标俯仰角和滚转角得到电机角度*********/
Eigen::Vector2d ParalleJoint::ik(double pitch, double roll)
{
    Eigen::Vector3d rc1 = xrot(pitch, roll) * rc01;
    Eigen::Vector3d rca1 = rc1 - ra01;
    Eigen::Vector2d motor;
    motor.setZero();

    double a1 = rca1(0);
    double b1 = -rca1(2);
    double c1 = (lrod1 * lrod1 - lbar * lbar - rca1.squaredNorm()) / (2 * lbar);

    double radicand1 = b1*b1*c1*c1 - (a1*a1 + b1*b1)*(c1*c1 - a1*a1);
    radicand1 = std::max(radicand1, 0.0);

    double theta1 =
        // asin((b1 * c1 + sqrt(b1 * b1 * c1 * c1 - (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1))) / (a1 * a1 + b1 * b1));
        asin((b1 * c1 + sqrt(radicand1)) / (a1 * a1 + b1 * b1));
    motor(0) = theta1;
    Eigen::Vector3d rc2 = xrot(pitch, roll) * rc02;
    Eigen::Vector3d rca2 = rc2 - ra02;

    double a2 = rca2(0);
    double b2 = -rca2(2);
    double c2 = (lrod2 * lrod2 - lbar * lbar - rca2.squaredNorm()) / (2 * lbar);
    double radicand2 = b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2);
    radicand2 = std::max(radicand2, 0.0);
    double theta2 =
        // asin((b2 * c2 + sqrt(b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2))) / (a2 * a2 + b2 * b2));
        asin((b2 * c2 + sqrt(radicand2)) / (a2 * a2 + b2 * b2));
    motor(1) = theta2;
    return motor;
}
/******输入俯仰角横滚角电机角度1，2，计算当前雅可比矩阵*******/
Eigen::Matrix2d ParalleJoint::Jac(double pitch, double roll, double theta1, double theta2, Eigen::Matrix2d &JcInv)
{
    Eigen::MatrixXd Jx(2, 6);
    Jx.setZero();
    Eigen::MatrixXd Jtheta(2, 2);
    Jtheta.setZero();
    Eigen::MatrixXd G(6, 2);
    G << 0, 0, 0, 0, 0, 0, cos(pitch), 0, 0, 1, -sin(pitch), 0;
    Eigen::Vector3d rb1 = ra01 + RyPlot(theta1) * rab01;
    Eigen::Vector3d rc1 = xrot(pitch, roll) * rc01;
    Eigen::Vector3d rbar1 = rb1 - ra01;
    Eigen::Vector3d rrod1 = rc1 - rb1;
    Eigen::Vector3d rb2 = ra02 + RyPlot(theta2) * rab02;
    Eigen::Vector3d rc2 = xrot(pitch, roll) * rc02;
    Eigen::Vector3d rbar2 = rb2 - ra02;
    Eigen::Vector3d rrod2 = rc2 - rb2;

    Eigen::Vector3d s11(0, 1, 0);
    Eigen::Vector3d s12(0, 1, 0);
    Jtheta(0, 0) = s11.dot(rbar1.cross(rrod1));
    Jtheta(1, 1) = s12.dot(rbar2.cross(rrod2));

    Jx.row(0).head(3) = rrod1.transpose();
    Jx.row(0).tail(3) = (rc1.cross(rrod1)).transpose();

    Jx.row(1).head(3) = rrod2.transpose();
    Jx.row(1).tail(3) = (rc2.cross(rrod2)).transpose();

    // Eigen::MatrixXd J_tem = Jx * G;
    // Eigen::Matrix2d Jc = Jtheta.inverse() * J_tem;

    // Eigen::Matrix2d Jc = Jtheta.inverse() * Jx * G;
    Eigen::Matrix2d Jc = Jtheta.ldlt().solve(Jx * G);
    double lambda = 1e-4; // 阻尼系数
    // JcInv = (Jc.transpose() * Jc + lambda * Eigen::Matrix2d::Identity()).inverse() * Jc.transpose();
    Eigen::Matrix2d damped = Jc.transpose() * Jc + lambda * Eigen::Matrix2d::Identity();
    JcInv = damped.ldlt().solve(Jc.transpose());

    // double lambda = 1e-4; // 阻尼系数
    // JcInv = J_tem.inverse() * Jtheta;
    // //(Jc.transpose() * Jc + lambda * Eigen::Matrix2d::Identity()).inverse() * Jc.transpose();
    // static bool startFlag = false;
    // if (JcInv.hasNaN())
    // {
    //     JcInv << 0, 0,
    //         0, 0;
    //     if (startFlag)
    //     {
    //         std::cout
    //             << " JcInv nan !!!!!!!!!" << std::endl;
    //         std::cout << " parallel  Motor     " << theta1 << "   "
    //                   << theta2 << "\n";
    //     }
    // }
    // else
    // {
    //     startFlag = true;
    // }
    return Jc;
}

Eigen::Vector2d ParalleJoint::fw(double pitchRef, double rollRef, double theta1, double theta2)
{
    const double epsilon = 1e-4;
    int max_iterations = 100;
    double dt = 1.0;
    // 0.51;
    double pitch = 0;
    double roll = 0;
    Eigen::Vector2d motor;
    motor.setZero();
    Eigen::Vector2d angle;
    angle.setZero();

    for (int i = 0; i < max_iterations; ++i)
    {
        Eigen::Matrix2d Jc = Eigen::Matrix2d::Zero();
        angle = ik(pitch, roll);

        Eigen::Vector2d error(angle(0) - theta1, angle(1) - theta2);
        if (error.norm() < epsilon) //&& v.norm() < epsilon
        {
            // std::cout << "SUCESESS: Inverse kinematics converged after " << motor << " iterations.\n";
            // motor(0) = pitch;
            // motor(1) = roll;
            // break;
            return motor;
        }

        // dt = std::min(1.0, error.norm() * 100.0);
        // if (error.norm() < epsilon) {
        //     // std::cout << "SUCESESS: Inverse kinematics converged after " << i << " iterations.\n";
        //     motor(0) = pitch;
        //     motor(1) = roll;
        //     break;
        // }
        Eigen::Matrix2d JcInv;
        if (std::isnan(angle(0)) || std::isnan(angle(1)))
        {
            // std::cout << angle(0) << " angle(0):  " << angle(1) << std::endl;
            break;
        }
        Jc = Jac(pitch, roll, angle(0), angle(1), JcInv);
        // std::cout << "Jc Right : "<< Jc <<std::endl;
        // Eigen::Matrix2d JcInv2;
        // bool invertible;
        // Jc.computeInverseWithCheck(JcInv2, invertible, std::numeric_limits<double>::epsilon());
        // if (!invertible)
        // {
        //     std::cout << "Jc inverse error :   ";
        //     // motor(0) = pitchRef; << std::endl
        //     // motor(1) = rollRef;
        //     std::cout << theta1 << " leg:  " << theta2 << std::endl;
        //     // return motor;
        // }
        // double lambda = 1e-4; // 阻尼系数
        // Eigen::Matrix2d JcInv = (Jc.transpose() * Jc + lambda * Eigen::Matrix2d::Identity()).inverse() *
        // Jc.transpose();

        Eigen::Vector2d v = JcInv * error;
        pitch -= v(1) * dt;
        roll -= v(0) * dt;
        motor(0) = pitch;
        motor(1) = roll;
    }
    motor(0) = pitchRef;
    motor(1) = rollRef;
    // dt = 0.5;
    // pitch = 0;
    // roll = 0;
    // motor.setZero();

    // for (int i = 0; i < max_iterations; ++i)
    // {
    //     Eigen::Matrix2d Jc = Eigen::Matrix2d::Zero();
    //     angle = ik(pitch, roll);

    //     Eigen::Vector2d error(angle(0) - theta1,
    //                           angle(1) - theta2);
    //     if (error.norm() < epsilon)
    //     {
    //         // std::cout << "SUCESESS: Inverse kinematics converged after " << motor << " iterations.\n";
    //         motor(0) = pitch;
    //         motor(1) = roll;
    //         return motor;
    //     }

    //     Eigen::Matrix2d JcInv;
    //     if (std::isnan(angle(0)) || std::isnan(angle(1)))
    //     {
    //         std::cout << angle(0) << " angle(0):  " << angle(1) << std::endl;
    //         motor[0] = 0;
    //         motor[1] = 0;
    //         return motor;
    //     }
    //     Jc = Jac(pitch, roll, angle(0), angle(1), JcInv);

    //     Eigen::Vector2d v = JcInv * error;
    //     pitch -= v(1) * dt;
    //     roll -= v(0) * dt;
    //     motor(0) = pitch;
    //     motor(1) = roll;
    // }
    // motor[0] = 0;
    // motor[1] = 0;
    // std::cout << "fw    errrr " << motor << " iterations.\n";
    // // motor(0) = pitchRef;
    // // motor(1) = rollRef;
    return motor;
}

void ParalleJoint::forceVelJoint2Motor(double pitch,
                                       double roll,
                                       double theta1,
                                       double theta2,
                                       const Eigen::Vector2d &jointVel,
                                       const Eigen::Vector2d &jointForce,
                                       Eigen::Vector2d &motorVel,
                                       Eigen::Vector2d &motorForce)
{
    // 获取雅可比矩阵
    Eigen::Matrix2d JcInv;
    Eigen::Matrix2d Jc = Jac(pitch, roll, theta1, theta2, JcInv);

    Eigen::Vector2d jointVelN(jointVel[1], jointVel[0]);
    // 计算末端速度
    motorVel = Jc * jointVelN;

    Eigen::Vector2d jointForceN(jointForce[1], jointForce[0]);
    // 计算关节力
    motorForce = JcInv.transpose() * jointForceN;
}

void ParalleJoint::forceVelMotor2Joint(double pitch,
                                       double roll,
                                       double theta1,
                                       double theta2,
                                       Eigen::Vector2d &jointVel,
                                       Eigen::Vector2d &jointForce,
                                       const Eigen::Vector2d &motorVel,
                                       const Eigen::Vector2d &motorForce)
{
    // 获取雅可比矩阵
    Eigen::Matrix2d JcInv;
    Eigen::Matrix2d Jc = Jac(pitch, roll, theta1, theta2, JcInv);

    // 计算末端速度
    Eigen::Vector2d jointVelN = JcInv * motorVel;
    jointVel << jointVelN[1], jointVelN[0];

    // 计算关节力
    Eigen::Vector2d jointForceN = Jc.transpose() * motorForce;
    jointForce << jointForceN[1], jointForceN[0];
}
