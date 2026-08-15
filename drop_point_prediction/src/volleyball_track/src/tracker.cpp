/*******************************************************************************
 * @file tracker.cpp
 * @brief 排球跟踪器实现，状态机：IDLE → DETECTING → TRACKING ↔ TEMP_LOST
 *******************************************************************************/

#include "volleyball_track/tracker.hpp"

namespace volleyball
{

/*****************************************************************
 * @brief 构造函数
 *****************************************************************/
Tracker::Tracker(std::unique_ptr<KalmanFilterBase> KF, int detect_thres, double lost_time_thres)
	: track_state_(TRACK_STATE_IDLE),
	  detect_thres_(detect_thres),
	  detect_cnt_(0),
	  lost_time_thres_(lost_time_thres),
	  lost_timer_(0.0)
{
	KF_ = std::move(KF);
	ball_update_state_ = Eigen::VectorXd::Zero(6);
}

/*****************************************************************
 * @brief 有观测：根据状态机执行初始化/确认/更新
 *****************************************************************/
void Tracker::update(const BallSharedPtr &ball_msg, double dt)
{
	Eigen::VectorXd measurement(3);
	measurement << ball_msg->x, ball_msg->y, ball_msg->z;

	switch (track_state_)
	{
	case TRACK_STATE_IDLE:
	{
		// 首次检测，初始化 Ekf 状态
		Eigen::VectorXd ball_state = Eigen::VectorXd::Zero(6);
		ball_state << ball_msg->x, 0, ball_msg->y, 0, std::max(0.0, static_cast<double>(ball_msg->z)), 0;
		KF_->set_state(ball_state);
		ball_update_state_ = ball_state;
		track_state_ = TRACK_STATE_DETECTING;
		detect_cnt_ = 1;
		lost_timer_ = 0.0;
		return;
	}

	case TRACK_STATE_DETECTING:
	{
		// 边跟踪边确认
		KF_->predict(dt);
		ball_update_state_ = KF_->update(measurement);
		detect_cnt_++;
		if (detect_cnt_ >= detect_thres_)
		{
			track_state_ = TRACK_STATE_TRACKING;
			lost_timer_ = 0.0;
		}
		return;
	}

	case TRACK_STATE_TRACKING:
	case TRACK_STATE_TEMP_LOST:
	{
		// 正常跟踪 / 丢检恢复
		KF_->predict(dt);
		ball_update_state_ = KF_->update(measurement);
		track_state_ = TRACK_STATE_TRACKING;
		lost_timer_ = 0.0;

		if (ball_update_state_(4) < 0.1)  // 0.1为排球半径
		{
			ball_update_state_(4) = 0.1;
			KF_->set_state(ball_update_state_);
		}
		return;
	}
	}
}

/*****************************************************************
 * @brief 无观测：Ekf 外推，超时则退回 IDLE
 *****************************************************************/
void Tracker::predict_only(double dt)
{
	switch (track_state_)
	{
	case TRACK_STATE_IDLE:
		return;

	case TRACK_STATE_DETECTING:
		// 确认阶段断开，视为 YOLO 误检，退回 IDLE
		track_state_ = TRACK_STATE_IDLE;
		detect_cnt_ = 0;
		lost_timer_ = 0.0;
		return;

	case TRACK_STATE_TRACKING:
		// 单帧丢失，开始外推
		ball_update_state_ = KF_->predict(dt);
		lost_timer_ = dt;
		track_state_ = TRACK_STATE_TEMP_LOST;
		return;

	case TRACK_STATE_TEMP_LOST:
		// 持续丢失，继续外推
		ball_update_state_ = KF_->predict(dt);
		lost_timer_ += dt;
		if (lost_timer_ > lost_time_thres_)
		{
			track_state_ = TRACK_STATE_IDLE;
			detect_cnt_ = 0;
			lost_timer_ = 0.0;
			RCLCPP_INFO(rclcpp::get_logger("tracker"), "球体丢失");
		}
		return;
	}
}

/*****************************************************************
 * @brief 复位至 IDLE
 *****************************************************************/
void Tracker::reset()
{
	track_state_ = TRACK_STATE_IDLE;
	detect_cnt_ = 0;
	lost_timer_ = 0.0;
}

}  // namespace volleyball
