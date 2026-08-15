/*******************************************************************************
 * @file tracker_node.hpp
 * @brief 跟踪节点类，订阅检测结果并发布滤波后的目标状态
 *******************************************************************************/

#ifndef VOLLEYBALL_TRACKER_NODE_HPP
#define VOLLEYBALL_TRACKER_NODE_HPP

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// tf2
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
// Project
#include "volleyball_interfaces/msg/ball.hpp"
#include "volleyball_track/KF.hpp"
#include "volleyball_track/measure_model.hpp"
#include "volleyball_track/process_model.hpp"
#include "volleyball_track/tracker.hpp"

namespace volleyball
{

/****************************************************************
 * @class TrackerNode 跟踪节点类
 ****************************************************************/
class TrackerNode : public rclcpp::Node
{
public:
	TrackerNode();

private:
	rclcpp::Time last_time_;
	rclcpp::Subscription<volleyball_interfaces::msg::Ball>::SharedPtr ball_sub_;
	rclcpp::Publisher<volleyball_interfaces::msg::Ball>::SharedPtr track_pub_;
	rclcpp::TimerBase::SharedPtr ball_lost_selfcheck_timer_;
	visualization_msgs::msg::Marker position_marker_;
	visualization_msgs::msg::Marker linear_v_marker_;
	visualization_msgs::msg::Marker ball_marker_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

	std::shared_ptr<MeasureModelContext> mm_ctx_;
	bool kf_initialized_ = false;
	bool initialing_ = false;
	bool camera_info_received_ = false;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
	double fx_, fy_;
	std::unique_ptr<Tracker> tracker_;

	std::string world_frame_;  // 目标世界坐标系
	std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

	double lost_self_check_time_thres_;  // 丢检自检阈值

	void try_init_kf();
	void ball_callback(const volleyball_interfaces::msg::Ball::SharedPtr msg);	   // 球检测回调
	void ball_lost_selfcheck_timer_callback();									   // 丢检自检定时回调
	void reset_tracker_callback(const std_srvs::srv::Trigger::Request::SharedPtr,  // 复位跟踪服务
								std_srvs::srv::Trigger::Response::SharedPtr response);
	void publish_tracked_ball(const rclcpp::Time &stamp);	 // 发布滤波结果及可视化
};

}  // namespace volleyball

#endif	// VOLLEYBALL_TRACKER_NODE_HPP
