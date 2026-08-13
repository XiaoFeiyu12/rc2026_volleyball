/*******************************************************************************
 * @file predictor.cpp
 * @brief 球轨迹预测器实现，基于线性空气阻力模型解析公式（与 Tracker 的 LinearDrag3D 一致）
 *******************************************************************************/

#include "volleyball_predict/predictor.hpp"

namespace volleyball
{
/*****************************************************************
 * @brief 构造函数
 * @param world_frame 世界坐标系名称
 * @param step 积分步长
 * @param k 空气阻力系数
 * @param m 排球质量
 * @param g 重力加速度
 *****************************************************************/
Predictor::Predictor(std::string world_frame, double step, double k, double m, double g)
  : step_(step), beta_(k / m), g_(g), world_frame_(world_frame)
{
}

/*****************************************************************
 * @brief 线性阻力模型单步预测（解析公式，与 makeLinearDrag3D::f 一致）
 * @param x 当前状态 [px, vx, py, vy, pz, vz]^T
 * @return 下一时刻状态
 *****************************************************************/
Eigen::VectorXd Predictor::step(const Eigen::VectorXd& x) const
{
    double phi = std::exp(-beta_ * step_);
    double n   = (1.0 - phi) / beta_;

    Eigen::VectorXd x_next(6);
    // x 轴
    x_next[0] = x[0] + n * x[1];
    x_next[1] = phi * x[1];
    // y 轴
    x_next[2] = x[2] + n * x[3]; 
    x_next[3] = phi * x[3];
    // z 轴（含重力）
    x_next[4] = x[4] + n * x[5] - g_ * (step_ - n) / beta_;
    x_next[5] = phi * x[5] - g_ * n;

    return x_next;
}

/*****************************************************************
 * @brief 预测球轨迹：从当前位置积分直至球落地（z<0）
 * @param ball_msg 球状态消息
 * @return 包含完整轨迹的BallTrajectory消息
 *****************************************************************/
volleyball_interfaces::msg::BallTrajectory Predictor::predict(volleyball_interfaces::msg::Ball::SharedPtr ball_msg)
{

    volleyball_interfaces::msg::BallTrajectory trajectory;
    // 获取球在世界坐标系下的状态
    Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
    state << ball_msg->x, ball_msg->vx, ball_msg->y, ball_msg->vy, ball_msg->z, ball_msg->vz;

    // 循环求解直至球落地
    auto stamp = ball_msg->header.stamp;
    uint64_t ns = static_cast<uint64_t>(stamp.sec) * 1'000'000'000ULL + stamp.nanosec;

    // 先存入当前状态（t=0）- 保证 z≥0
    {
        volleyball_interfaces::msg::Ball point;
        point.x = state[0];
        point.vx = state[1];
        point.y = state[2];
        point.vy = state[3];
        point.z = std::max(0.0, state[4]);
        point.vz = state[5];
        point.header.frame_id = world_frame_;
        point.header.stamp.sec = ns / 1'000'000'000ULL;
        point.header.stamp.nanosec = ns % 1'000'000'000ULL;
        trajectory.ball_trajectory.emplace_back(point);
    }

    // 用线性阻力解析公式外推（存点前先检查 z≥0）
    while (true)
    {
        state = step(state);
        if (state[4] < 0.0)
            break;  // 球已落地，不存低于地面的点
        ns += static_cast<uint64_t>(step_ * 1e9);
        volleyball_interfaces::msg::Ball point;
        point.x = state[0];
        point.vx = state[1];
        point.y = state[2];
        point.vy = state[3];
        point.z = state[4];
        point.vz = state[5];
        point.header.frame_id = world_frame_;
        point.header.stamp.sec = ns / 1'000'000'000ULL;
        point.header.stamp.nanosec = ns % 1'000'000'000ULL;
        trajectory.ball_trajectory.emplace_back(point);
    }

    trajectory.header.frame_id = world_frame_;
    trajectory.header.stamp = ball_msg->header.stamp;

    return trajectory;
}
}  // namespace volleyball
