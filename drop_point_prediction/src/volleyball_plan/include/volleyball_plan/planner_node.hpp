#ifndef __PLANNER_NODE_HPP
#define __PLANNER_NODE_HPP

// stl
#include <cmath>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// eigen
#include <eigen3/Eigen/Dense>
// Project
#include "volleyball_interfaces/msg/ball_trajectory.hpp"
#include "volleyball_interfaces/msg/plan.hpp"
#include "volleyball_plan/planner.hpp"

namespace volleyball
{
class PlannerNode : public rclcpp::Node
{
public:
	PlannerNode();

private:
	// striking_point_bias_ = [dx, dy, dz]^T
	Eigen::Vector3d striking_point_bias_;

	std::shared_ptr<Planner> planner_;
	rclcpp::Publisher<volleyball_interfaces::msg::Plan>::SharedPtr plan_pub_;
	rclcpp::Subscription<volleyball_interfaces::msg::BallTrajectory>::SharedPtr ball_trajectory_sub_;

	visualization_msgs::msg::Marker plan_point_marker_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

	void ball_trajectory_callback(const BallTrajectory::SharedPtr msg);
};
}  // namespace volleyball

#endif