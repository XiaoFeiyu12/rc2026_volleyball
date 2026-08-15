/*******************************************************************************
 * @file tracker.hpp
 * @brief 排球跟踪器类，管理卡尔曼滤波器的预测与更新
 *******************************************************************************/

#ifndef VOLLEYBALL_TRACKER_HPP
#define VOLLEYBALL_TRACKER_HPP

// ROS2
#include "rclcpp/rclcpp.hpp"
// Eigen
#include <eigen3/Eigen/Dense>
// Project
#include "volleyball_interfaces/msg/ball.hpp"
#include "volleyball_track/KF.hpp"

namespace volleyball
{

/****************************************************************
 * @class Tracker 排球跟踪器类
 * @brief 单目标 Ekf 跟踪器，状态机：IDLE → DETECTING → TRACKING ↔ TEMP_LOST
 *
 *   ┌──────────┐
 *   │   IDLE   │◄──── 连续丢失 > lost_time_thres ──────┐
 *   └────┬─────┘                                       │
 *        │ 首次检测，set_state                          │
 *        ▼                                             │
 *   ┌──────────┐                                       │
 *   │DETECTING │ (连续 detect_thres 帧 → TRACKING)      │
 *   └────┬─────┘                                       │
 *        │ 确认                                       │
 *        ▼                                             │
 *   ┌──────────┐      50ms未有新数据          ┌───────────┐
 *   │ TRACKING │◄──────────────────────────►│ TEMP_LOST │
 *   └──────────┘      检测恢复               └─────┬─────┘
 ****************************************************************/
class Tracker
{
public:
	using Ball = volleyball_interfaces::msg::Ball;
	using BallSharedPtr = volleyball_interfaces::msg::Ball::SharedPtr;

	typedef enum
	{
		TRACK_STATE_IDLE,		// 无目标
		TRACK_STATE_DETECTING,	// 检测确认中（防 YOLO 误检）
		TRACK_STATE_TRACKING,	// 正常跟踪
		TRACK_STATE_TEMP_LOST	// 短暂丢失，Ekf 外推中
	} TrackState;

	Tracker(std::unique_ptr<KalmanFilterBase> KF, int detect_thres, double lost_time_thres);
	void update(const BallSharedPtr &ball_msg, double dt);	// 有观测：predict + update
	void predict_only(double dt);							// 无观测：仅 predict 外推
	TrackState get_state() const { return track_state_; }
	void reset();  // 复位至 IDLE
	Eigen::VectorXd get_ball_position() { return ball_update_state_; }

private:
	std::unique_ptr<KalmanFilterBase> KF_;	// 卡尔曼滤波器
	Eigen::VectorXd ball_update_state_;		// 当前滤波后状态
	TrackState track_state_;				// 当前状态机
	int detect_thres_;						// 连续检测确认帧数
	int detect_cnt_;						// 当前连续计数
	double lost_time_thres_;				// 丢检超时阈值
	double lost_timer_;						// 累计丢检时长
};

}  // namespace volleyball

#endif	// VOLLEYBALL_TRACKER_HPP
