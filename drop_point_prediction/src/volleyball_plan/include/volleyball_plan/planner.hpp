#ifndef __PLANNER_HPP
#define __PLANNER_HPP

// ROS2
#include "rclcpp/time.hpp"
// eigen
#include <eigen3/Eigen/Dense>
// Project
#include "volleyball_interfaces/msg/ball_trajectory.hpp"
#include "volleyball_interfaces/msg/plan.hpp"

namespace volleyball
{

using Plan = volleyball_interfaces::msg::Plan;
using BallTrajectory = volleyball_interfaces::msg::BallTrajectory;

class Planner
{
public:
	Planner();
	Plan plan(const BallTrajectory::SharedPtr msg, Eigen::Vector3d striking_bias, rclcpp::Time now);

private:
};
}  // namespace volleyball

#endif