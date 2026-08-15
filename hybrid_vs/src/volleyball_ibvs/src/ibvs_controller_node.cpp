/*******************************************************************************
 * @file ibvs_controller_node.cpp
 * @brief IBVS 控制器节点实现（含 TF2 坐标变换 + EMA 低通滤波）
 *******************************************************************************/

#include "volleyball_ibvs/ibvs_controller_node.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace volleyball
{

IbvsControllerNode::IbvsControllerNode()
  : Node("ibvs_controller_node"),
    last_y_(0.0),
    smoothed_dist_(0.0),
    smoothed_y_(0.0),
    filter_inited_(false)
{
  // ── 参数 ──
  this->declare_parameter<double>("Kp_lat", 1.0);
  this->declare_parameter<double>("Kp_long", 0.8);
  this->declare_parameter<double>("alpha", 0.4);
  this->declare_parameter<double>("timeout", 0.5);
  this->declare_parameter<std::string>("target_frame", "base_link");
  this->declare_parameter<double>("Kd_lat", 0.5);
  this->declare_parameter<double>("dist_offset", 0.5);
  this->declare_parameter<double>("hit_offset_x", 0.2);
  this->declare_parameter<double>("hit_offset_y", 0.2);
  this->declare_parameter<double>("hit_offset_z", 0.2);

  this->get_parameter("Kp_lat", Kp_lat_);
  this->get_parameter("Kp_long", Kp_long_);
  this->get_parameter("alpha", alpha_);
  this->get_parameter("timeout", timeout_);
  this->get_parameter("target_frame", target_frame_);
  this->get_parameter("Kd_lat", Kd_lat_);
  this->get_parameter("dist_offset", dist_offset_);
  this->get_parameter("hit_offset_x", hit_offset_x_);
  this->get_parameter("hit_offset_y", hit_offset_y_);
  this->get_parameter("hit_offset_z", hit_offset_z_);

  last_stamp_ = this->now();

  // ── TF2 ──
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── 订阅 detector 的 3D 球位置 ──
  ball_sub_ = this->create_subscription<Ball>(
      "/detector/ball", rclcpp::SensorDataQoS(),
      std::bind(&IbvsControllerNode::ball_callback, this, std::placeholders::_1));

  // ── 发布到 /pid_camera（serial_driver 已订阅，复用 cmd=1） ──
  pid_camera_pub_ = this->create_publisher<PidCamera>(
      "/pid_camera", rclcpp::SensorDataQoS());

  // ── 看门狗定时器 ──
  watchdog_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timeout_ / 2.0),
      std::bind(&IbvsControllerNode::watchdog_callback, this));

  last_ball_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
      "IBVS 控制器启动 | target_frame=%s | Kp_lat=%.2f Kp_long=%.2f "
      "alpha=%.2f timeout=%.2fs",
      target_frame_.c_str(),
      Kp_lat_, Kp_long_, alpha_, timeout_);
}

void IbvsControllerNode::ball_callback(const Ball::SharedPtr msg)
{
  last_ball_time_ = this->now();

  // ── TF 变换: camera_color_optical_frame → strike_point ──
  geometry_msgs::msg::PointStamped pt_cam;
  pt_cam.header = msg->header;
  pt_cam.point.x = msg->x;
  pt_cam.point.y = msg->y;
  pt_cam.point.z = msg->z;

  geometry_msgs::msg::PointStamped pt_base;
  try
  {
    pt_base = tf_buffer_->transform(pt_cam, target_frame_);
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "TF 变换失败 (%s → %s): %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
    return;
  }

  RCLCPP_DEBUG(this->get_logger(),
      "球位置 (strike_point): x=%.3f y=%.3f z=%.3f",
      pt_base.point.x, pt_base.point.y, pt_base.point.z);

  double dist = pt_base.point.x + dist_offset_;
  double y    = - pt_base.point.y;

  // ── EMA 低通滤波 ──
  rclcpp::Time stamp(msg->header.stamp);
  double dt = (stamp - last_stamp_).seconds();
  last_stamp_ = stamp;

  if (!filter_inited_)
  {
    smoothed_dist_ = dist;
    smoothed_y_    = y;
    filter_inited_  = true;
  }
  else
  {
    smoothed_dist_ = alpha_ * dist + (1.0 - alpha_) * smoothed_dist_;
    smoothed_y_    = alpha_ * y    + (1.0 - alpha_) * smoothed_y_;
  }

  double dy_diff = (smoothed_y_ - last_y_) / std::max(dt, 0.01);
  last_y_ = smoothed_y_;

  // ── 横向: 位置偏差 PD (strike_point 系)  纵向: 距离比例 ──
  double vy_cmd = Kp_lat_  * smoothed_y_ + Kd_lat_ * dy_diff;
  double vx_cmd = Kp_long_ * smoothed_dist_;

  PidCamera pid_msg;
  pid_msg.pixel_diff_x = static_cast<float>(vy_cmd) ;
  pid_msg.pixel_diff_y = static_cast<float>(vx_cmd) ; // 放大以便下位机处理

  // 处理击球逻辑
  if (pt_base.point.x < hit_offset_x_ && pt_base.point.x > 0.0 && std::abs(pt_base.point.y) < hit_offset_y_ && std::abs(pt_base.point.z) < hit_offset_z_)
  {
    pid_msg.is_hit = 1;
  }
  else
  {
    pid_msg.is_hit = 0;
  }

  pid_camera_pub_->publish(pid_msg);

}


void IbvsControllerNode::watchdog_callback()
{
  double dt = (this->now() - last_ball_time_).seconds();
  if (dt > timeout_ && filter_inited_)
  {
    PidCamera pid_msg;
    pid_msg.pixel_diff_x = 0.0f;
    pid_msg.pixel_diff_y = 0.0f;
    pid_camera_pub_->publish(pid_msg);
    filter_inited_ = false;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "检测丢失 %.1fs → 零速", dt);
  }
}

}  // namespace volleyball
