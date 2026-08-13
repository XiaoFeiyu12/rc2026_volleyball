/*******************************************************************************
 * @file ibvs_controller_node.hpp
 * @brief IBVS 反应式控制器
 *
 * 控制律 (输出 cm/s):
 *   横向: vy_cmd = Kp_lat * y + Kd_lat * Δy   (y单位m, strike_point系)
 *   纵向: vx_cmd = Kp_long * dist              (dist单位m, Kp_long单位(cm/s)/m)
 *
 * 流程:
 *   detector Ball (camera_color_optical_frame)
 *     → TF 变换到 strike_point (取 dist=x, y)
 *     → EMA 低通滤波 (dist, y)
 *     → 横向 PD / 纵向 P 输出
 *******************************************************************************/

#ifndef IBVS_CONTROLLER_NODE_HPP
#define IBVS_CONTROLLER_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "volleyball_interfaces/msg/ball.hpp"
#include "volleyball_interfaces/msg/pid_camera.hpp"

namespace volleyball
{

class IbvsControllerNode : public rclcpp::Node
{
public:
  IbvsControllerNode();

private:
  using Ball = volleyball_interfaces::msg::Ball;
  using PidCamera = volleyball_interfaces::msg::PidCamera;

  void ballCallback(const Ball::SharedPtr msg);
  void watchdogCallback();


  // ── TF ──
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── 订阅 / 发布 ──
  rclcpp::Subscription<Ball>::SharedPtr ball_sub_;
  rclcpp::Publisher<PidCamera>::SharedPtr pid_camera_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // ── 参数 ──
  double Kp_lat_;
  double Kp_long_;
  double alpha_;
  double timeout_;
  std::string target_frame_;  // 默认 "base_link"
  double last_y_;
  double Kd_lat_;
  double dist_offset_;
  double hit_offset_x_;  // 击球点 X 轴范围，单位 m
  double hit_offset_y_;  // 击球点 Y 轴范围，单位 m
  double hit_offset_z_;  // 击球点 Z 轴范围，单位 m

  // ── EMA 滤波状态 ──
  double smoothed_dist_;   // 平滑后水平前方距离
  double smoothed_y_;      // 平滑后横向偏移 (m, strike_point 系)
  bool   filter_inited_;
  rclcpp::Time last_ball_time_;
  rclcpp::Time last_stamp_;  // 上一帧时间戳，用于 D 项 dt 归一化
};

}  // namespace volleyball
#endif
