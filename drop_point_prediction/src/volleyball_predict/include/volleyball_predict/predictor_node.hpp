/*******************************************************************************
 * @file predictor_node.hpp
 * @brief 预测节点类，订阅检测到的球并发布预测轨迹
 *******************************************************************************/

#ifndef PREDICTOR_NODE_HPP
#define PREDICTOR_NODE_HPP

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// STL
#include <vector>
// eigen
#include <eigen3/Eigen/Dense>
// Project
#include "volleyball_interfaces/msg/ball.hpp"
#include "volleyball_interfaces/msg/ball_trajectory.hpp"
#include "volleyball_predict/predictor.hpp"

namespace volleyball
{

/****************************************************************
 * @class PredictNode 预测节点类
 ****************************************************************/
class PredictNode : public rclcpp::Node
{
public:
	typedef enum
	{
		TO_OWN_SIDE = 0,   // 球飞向己方
		TO_OPPONENT_SIDE,  // 球飞向对方
		TO_STOP,		   // 球静止
	} VolleyballState;

	PredictNode();

private:
	double lost_time_thres_;  // 丢检超时阈值
	std::string odom_frame_id_;
	std::shared_ptr<Predictor> predictor_;
	volleyball_interfaces::msg::Ball::SharedPtr ball_cache_;  // 球消息缓存
	visualization_msgs::msg::Marker ball_trajectory_point_marker_;
	visualization_msgs::msg::Marker landing_point_marker_;

	rclcpp::Subscription<volleyball_interfaces::msg::Ball>::SharedPtr target_ball_sub_;
	rclcpp::Publisher<volleyball_interfaces::msg::BallTrajectory>::SharedPtr ball_trajectory_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_marker_pub_;
	rclcpp::TimerBase::SharedPtr predict_trajectory_timer_;

	void target_ball_sub_callback(const volleyball_interfaces::msg::Ball::SharedPtr msg);	 // 球消息订阅回调
	void predict_trajectory_timer_callback();												 // 定时预测回调
	void publish_trajectory_marker(const volleyball_interfaces::msg::BallTrajectory msg);	 // 发布轨迹可视化
};

}  // namespace volleyball

#endif	// PREDICTOR_NODE_HPP
