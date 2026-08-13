#include "volleyball_plan/planner.hpp"

namespace volleyball
{
Planner::Planner()
{
}

Plan Planner::plan(const BallTrajectory::SharedPtr msg, Eigen::Vector3d striking_bias, rclcpp::Time now)
{
    Plan plan;
    // 轨迹为空则直接返回
    if (msg->ball_trajectory.empty())
    {
        return plan;
    }
    // 从落点开始反向寻找机械臂可以击中的点
    Eigen::Vector3d diff(3);
    diff << 0, 0, -1;
    // 查找轨迹点减去击打点相对base_Link的偏移量还在地面以上的点
    volleyball_interfaces::msg::Ball point;
    // 拷贝一份
    auto ball_trajectory = std::vector<volleyball_interfaces::msg::Ball>(msg->ball_trajectory);
    while (!ball_trajectory.empty() && diff.z() <= 0.0f)
    {
        point = ball_trajectory.back();
        ball_trajectory.pop_back();
        Eigen::Vector3d point_vec;
        point_vec << point.x, point.y, point.z;
        diff = point_vec - striking_bias;
    }

    // striking_point = [dx, dy, dz]^T
    plan.x = point.x + striking_bias[0];
    plan.y = point.y + striking_bias[1];
    plan.landing_time = (rclcpp::Time(point.header.stamp) - now).seconds();
    // 目前接法是直接对着对方打过去，不考虑角度
    plan.self_yaw = 0.0;

    return plan;
}

}  // namespace volleyball