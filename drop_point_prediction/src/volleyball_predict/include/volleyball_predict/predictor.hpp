/*******************************************************************************
 * @file predictor.hpp
 * @brief 球轨迹预测器类，基于线性空气阻力模型解析公式预测排球轨迹
 *******************************************************************************/

#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

// ROS2
#include "geometry_msgs/msg/point_stamped.hpp"
// eigen
#include <eigen3/Eigen/Dense>
// Project
#include "volleyball_interfaces/msg/ball.hpp"
#include "volleyball_interfaces/msg/ball_trajectory.hpp"

namespace volleyball {

/****************************************************************
 * @class Predictor 球轨迹预测器类
 ****************************************************************/
class Predictor
{
private:
    double step_;   // 轨迹求解步长
    double beta_;   // 线性阻力系数 beta = k/m
    double g_;      // 重力加速度
    std::string world_frame_;

    // 线性阻力模型单步预测（解析公式，与 LinearDrag3D 一致）
    Eigen::VectorXd step(const Eigen::VectorXd& x) const;

public:
    Predictor(std::string world_frame, double step, double k, double m, double g = 9.80);
    // 预测球轨迹，返回从当前位置到落地的完整轨迹
    volleyball_interfaces::msg::BallTrajectory predict(volleyball_interfaces::msg::Ball::SharedPtr ball_msg);
};

}  // namespace volleyball

#endif  // PREDICTOR_HPP
