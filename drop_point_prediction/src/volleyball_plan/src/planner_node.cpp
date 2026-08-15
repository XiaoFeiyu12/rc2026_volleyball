#include "volleyball_plan/planner_node.hpp"

namespace volleyball
{
PlannerNode::PlannerNode() : Node("planner_node")
{
	// 先简陋的直接计算偏差，后续改为从tf树中得到击打点偏差向量
	this->declare_parameter<double>("base_to_armjoint", 0.18);
	this->declare_parameter<double>("armjoint_to_strikingpoint", 0.25);
	this->declare_parameter<double>("delta_arm_angle", 33.0);

	double base_to_armjoint = this->get_parameter("base_to_armjoint").as_double();
	double armjoint_to_strikingpoint = this->get_parameter("armjoint_to_strikingpoint").as_double();
	double delta_arm_angle = this->get_parameter("delta_arm_angle").as_double();

	striking_point_bias_ << armjoint_to_strikingpoint * sin(delta_arm_angle / 180.0 * 3.1415926), 0,
		armjoint_to_strikingpoint * cos(delta_arm_angle / 180.0 * 3.1415926) + base_to_armjoint;

	planner_ = std::make_shared<Planner>();

	plan_pub_ = this->create_publisher<Plan>("/planner/plan", rclcpp::SensorDataQoS());
	ball_trajectory_sub_ = this->create_subscription<BallTrajectory>(
		"/predictor/ball_trajectory", rclcpp::SensorDataQoS(),
		std::bind(&PlannerNode::ball_trajectory_callback, this, std::placeholders::_1));

	marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/marker", 10);
	plan_point_marker_.ns = "plan_point";
	plan_point_marker_.type = visualization_msgs::msg::Marker::ARROW;
	plan_point_marker_.action = visualization_msgs::msg::Marker::ADD;
	plan_point_marker_.lifetime = rclcpp::Duration(1, 0);
	plan_point_marker_.scale.x = plan_point_marker_.scale.y = plan_point_marker_.scale.z = 0.1;
	plan_point_marker_.color.a = 1.0;
	plan_point_marker_.color.b = 1.0;

	RCLCPP_INFO(this->get_logger(), "planner_node初始化完成！");
}

void PlannerNode::ball_trajectory_callback(const BallTrajectory::SharedPtr msg)
{
	auto plan = planner_->plan(msg, striking_point_bias_, this->now());
	plan_pub_->publish(plan);
	RCLCPP_INFO(this->get_logger(), "发布规划消息");
	{
		plan_point_marker_.header.frame_id = msg->header.frame_id;
		plan_point_marker_.header.stamp = msg->header.stamp;
		plan_point_marker_.points.clear();
		geometry_msgs::msg::Point arrow_start;
		arrow_start.x = plan.x;
		arrow_start.y = plan.y;
		arrow_start.z = 0.0;
		plan_point_marker_.points.emplace_back(arrow_start);
		geometry_msgs::msg::Point arrow_end = arrow_start;
		const double ARROW_SCALE = 1;
		arrow_end.x += cos(plan.self_yaw * 3.1415926 / 180.0) * ARROW_SCALE;
		arrow_end.y += sin(plan.self_yaw * 3.1415926 / 180.0) * ARROW_SCALE;
		plan_point_marker_.points.emplace_back(arrow_end);
		visualization_msgs::msg::MarkerArray marker_array;
		marker_array.markers.emplace_back(plan_point_marker_);
		marker_pub_->publish(marker_array);
	}
}

}  // namespace volleyball