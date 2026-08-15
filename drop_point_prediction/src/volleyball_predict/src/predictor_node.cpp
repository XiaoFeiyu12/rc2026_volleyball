/*******************************************************************************
 * @file predictor_node.cpp
 * @brief 预测节点实现，订阅球检测结果并发布预测轨迹
 *******************************************************************************/

#include "volleyball_predict/predictor_node.hpp"

namespace volleyball
{
/*****************************************************************
 * @brief 构造函数：加载参数、初始化预测器与ROS组件
 *****************************************************************/
PredictNode::PredictNode() : Node("predict_node")
{
	this->declare_parameter<int>("predict_timer_freq", 40);
	this->declare_parameter<std::string>("odom_frame_id", "odom");
	this->declare_parameter<double>("predict_step", 0.1);
	this->declare_parameter<double>("k", 0.10);
	this->declare_parameter<double>("m", 0.27);
	this->declare_parameter<double>("g", 9.80);
	this->declare_parameter<double>("lost_time_thres", 3.0);

	int predict_timer_freq;
	double k, m, g, predict_step;

	predict_timer_freq = this->get_parameter("predict_timer_freq").as_int();
	odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
	predict_step = this->get_parameter("predict_step").as_double();
	k = this->get_parameter("k").as_double();
	m = this->get_parameter("m").as_double();
	g = this->get_parameter("g").as_double();
	lost_time_thres_ = this->get_parameter("lost_time_thres").as_double();

	ball_trajectory_point_marker_.ns = "ball_trajectory";
	ball_trajectory_point_marker_.type = visualization_msgs::msg::Marker::SPHERE;
	ball_trajectory_point_marker_.action = visualization_msgs::msg::Marker::ADD;
	ball_trajectory_point_marker_.lifetime = rclcpp::Duration(5, 0);
	ball_trajectory_point_marker_.scale.x = ball_trajectory_point_marker_.scale.y =
		ball_trajectory_point_marker_.scale.z = 0.08;
	ball_trajectory_point_marker_.color.a = 1.0;
	ball_trajectory_point_marker_.color.r = 1.0;
	ball_trajectory_point_marker_.color.b = 1.0;

	landing_point_marker_.ns = "landing_point";
	landing_point_marker_.type = visualization_msgs::msg::Marker::CUBE;
	landing_point_marker_.action = visualization_msgs::msg::Marker::ADD;
	landing_point_marker_.lifetime = rclcpp::Duration(1, 0);
	landing_point_marker_.scale.x = landing_point_marker_.scale.y = 0.2;
	landing_point_marker_.scale.z = 0.05;
	landing_point_marker_.color.a = 1.0;
	landing_point_marker_.color.g = 1.0;

	target_ball_sub_ = this->create_subscription<volleyball_interfaces::msg::Ball>(
		"/tracker/target", rclcpp::SensorDataQoS(),
		std::bind(&PredictNode::target_ball_sub_callback, this, std::placeholders::_1));

	ball_trajectory_pub_ = this->create_publisher<volleyball_interfaces::msg::BallTrajectory>(
		"/predictor/ball_trajectory", rclcpp::SensorDataQoS());

	trajectory_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/predictor/marker", 10);

	predict_trajectory_timer_ =
		this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0f / predict_timer_freq)),
								std::bind(&PredictNode::predict_trajectory_timer_callback, this));

	predictor_ = std::make_shared<Predictor>(odom_frame_id_, predict_step, k, m, g);

	RCLCPP_INFO(this->get_logger(), "predictor_node初始化完成");
}

/*****************************************************************
 * @brief 球消息订阅回调：缓存球状态
 *****************************************************************/
void PredictNode::target_ball_sub_callback(const volleyball_interfaces::msg::Ball::SharedPtr msg)
{
	// 拷贝一份
	ball_cache_ = std::make_shared<volleyball_interfaces::msg::Ball>(*msg);
}

/*****************************************************************
 * @brief 预测定时回调：检查丢检状态，对飞往己方的球进行轨迹预测
 *****************************************************************/
void PredictNode::predict_trajectory_timer_callback()
{
	if (!ball_cache_)
	{
		return;
	}

	auto now = this->now();
	auto dt = (now - rclcpp::Time(ball_cache_->header.stamp)).seconds();
	if (dt > lost_time_thres_)
	{
		// 如果上一帧球消息缓存时间过长则认为丢检
		ball_cache_.reset();
		RCLCPP_WARN(this->get_logger(), "预测器丢检。");
		return;
	}

	volleyball_interfaces::msg::BallTrajectory trajectory;
	trajectory = predictor_->predict(ball_cache_);
	ball_trajectory_pub_->publish(trajectory);
	publish_trajectory_marker(trajectory);
	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "发布轨迹: %zu个点, 落点x=%.1f,y=%.1f",
						 trajectory.ball_trajectory.size(),
						 trajectory.ball_trajectory.empty() ? 0.0 : trajectory.ball_trajectory.back().x,
						 trajectory.ball_trajectory.empty() ? 0.0 : trajectory.ball_trajectory.back().y);

	auto end = this->now();
	auto time = end - now;
	RCLCPP_DEBUG(this->get_logger(), "预测耗费时间：%fs", time.seconds());
}

/*****************************************************************
 * @brief 发布预测轨迹可视化标记
 *****************************************************************/
void PredictNode::publish_trajectory_marker(const volleyball_interfaces::msg::BallTrajectory msg)
{
	visualization_msgs::msg::MarkerArray marker_array;
	int i = 0;
	for (auto trajectory_point : msg.ball_trajectory)
	{
		ball_trajectory_point_marker_.header.frame_id = odom_frame_id_;
		ball_trajectory_point_marker_.header.stamp = this->now();
		ball_trajectory_point_marker_.id = i++;
		ball_trajectory_point_marker_.pose.position.x = trajectory_point.x;
		ball_trajectory_point_marker_.pose.position.y = trajectory_point.y;
		ball_trajectory_point_marker_.pose.position.z = trajectory_point.z;
		marker_array.markers.emplace_back(ball_trajectory_point_marker_);
	}

	// 添加落点大标记（轨迹最后一个点）
	if (!msg.ball_trajectory.empty())
	{
		auto &last = msg.ball_trajectory.back();
		landing_point_marker_.header.frame_id = odom_frame_id_;
		landing_point_marker_.header.stamp = this->now();
		landing_point_marker_.id = 9999;
		landing_point_marker_.pose.position.x = last.x;
		landing_point_marker_.pose.position.y = last.y;
		landing_point_marker_.pose.position.z = 0.0;  // 落在地上
		marker_array.markers.emplace_back(landing_point_marker_);
	}

	trajectory_marker_pub_->publish(marker_array);
}

}  // namespace volleyball
