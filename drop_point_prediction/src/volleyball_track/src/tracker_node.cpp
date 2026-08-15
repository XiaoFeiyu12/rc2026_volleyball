/*******************************************************************************
 * @file tracker_node.cpp
 * @brief 跟踪节点实现，订阅球检测结果、坐标变换、发布滤波后目标状态
 *******************************************************************************/

#include "volleyball_track/tracker_node.hpp"

#include <string>

namespace volleyball
{
/*****************************************************************
 * @brief 构造函数：加载参数、初始化滤波器与ROS组件
 *****************************************************************/
TrackerNode::TrackerNode() : Node("tracker_node")
{
	camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		"/camera/camera/color/camera_info", rclcpp::QoS(1).reliable(),
		[this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
		{
			fx_ = msg->k[0];
			fy_ = msg->k[4];
			camera_info_received = true;
			try_init_kf();
		});

	RCLCPP_INFO(this->get_logger(), "等待获取相机参数");
}

/*****************************************************************
 * @brief 供获取相机参数后初始化节点
 *****************************************************************/
void TrackerNode::try_init_kf()
{
	if (kf_initialized_ || initialing_) return;
	initialing_ = true;
	// 物理模型参数
	this->declare_parameter<double>("k", 0.20);
	this->declare_parameter<double>("m", 0.27);
	this->declare_parameter<double>("g", 9.80);
	double k = this->get_parameter("k").as_double();
	double m = this->get_parameter("m").as_double();
	double g = this->get_parameter("g").as_double();

	// 过程噪声矩阵（自适应Q：低速大→跟测量，高速小→信任模型）
	this->declare_parameter<double>("Q_sigma_xy_min", 10.0);
	this->declare_parameter<double>("Q_sigma_xy_max", 300.0);
	this->declare_parameter<double>("Q_sigma_z_min", 15.0);
	this->declare_parameter<double>("Q_sigma_z_max", 400.0);
	double Q_sigma_xy_min = this->get_parameter("Q_sigma_xy_min").as_double();
	double Q_sigma_xy_max = this->get_parameter("Q_sigma_xy_max").as_double();
	double Q_sigma_z_min = this->get_parameter("Q_sigma_z_min").as_double();
	double Q_sigma_z_max = this->get_parameter("Q_sigma_z_max").as_double();

	// 测量噪声矩阵
	this->declare_parameter<int>("sigma_pixel", 2);
	this->declare_parameter<double>("sigma_depth_gain", 0.04);
	this->declare_parameter<double>("sigma_depth_const", 0.05);
	int sigma_pixel = this->get_parameter("sigma_pixel").as_int();
	double sigma_depth_gain = this->get_parameter("sigma_depth_gain").as_double();
	double sigma_depth_const = this->get_parameter("sigma_depth_const").as_double();

	// tracker各种时间阈值
	this->declare_parameter<int>("detect_cnt_thres", 3);
	this->declare_parameter<double>("lost_time_thres", 0.5);
	this->declare_parameter<double>("selfcheck_time_thres", 0.05);
	int detect_cnt_thres = this->get_parameter("detect_cnt_thres").as_int();
	double lost_time_thres = this->get_parameter("lost_time_thres").as_double();
	double selfcheck_time_thres = this->get_parameter("selfcheck_time_thres").as_double();

	// 初始化卡尔曼滤波器
	mm_ctx = std::make_shared<MeasureModelContext>();
	auto mm = make_position_3d(sigma_pixel, sigma_depth_gain, sigma_depth_const, fx_, fy_, mm_ctx);
	auto pm = make_linear_drag_3d(Q_sigma_xy_min, Q_sigma_xy_max, Q_sigma_z_min, Q_sigma_z_max, k, m, g);
	Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6);
	auto ekf = std::make_unique<Ekf>(P0, std::move(pm), std::move(mm));
	tracker_ = std::make_unique<Tracker>(std::move(ekf), detect_cnt_thres, lost_time_thres);
	kf_initialized_ = true;

	// ROS2相关订阅发布
	ball_sub_ = this->create_subscription<volleyball_interfaces::msg::Ball>(
		"/detector/ball", rclcpp::SensorDataQoS(), std::bind(&TrackerNode::ball_callback, this, std::placeholders::_1));
	track_pub_ = this->create_publisher<volleyball_interfaces::msg::Ball>("/tracker/target", rclcpp::SensorDataQoS());
	ball_lost_selfcheck_timer_ =
		this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(selfcheck_time_thres * 1000)),
								std::bind(&TrackerNode::ball_lost_selfcheck_timer_callback, this));
	reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
		"/tracker/reset",
		std::bind(&TrackerNode::reset_tracker_callback, this, std::placeholders::_1, std::placeholders::_2));
	// tf2相关
	world_frame_ = this->declare_parameter("target_frame_id", "odom");
	tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	auto timer_interface =
		std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
	tf2_buffer_->setCreateTimerInterface(timer_interface);
	tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
	// 可视化
	marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
	position_marker_.ns = "position";
	position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
	position_marker_.action = visualization_msgs::msg::Marker::ADD;
	position_marker_.lifetime = rclcpp::Duration(6, 0);
	position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
	position_marker_.color.a = 1.0;
	position_marker_.color.g = 1.0;
	linear_v_marker_.ns = "linear_v";
	linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
	linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
	linear_v_marker_.lifetime = rclcpp::Duration(0, 500000000);
	linear_v_marker_.scale.x = linear_v_marker_.scale.y = linear_v_marker_.scale.z = 0.05;
	linear_v_marker_.color.a = 1.0;
	linear_v_marker_.color.b = 1.0;
	linear_v_marker_.color.g = 1.0;
	ball_marker_.ns = "ball";
	ball_marker_.type = visualization_msgs::msg::Marker::SPHERE;
	ball_marker_.action = visualization_msgs::msg::Marker::ADD;
	ball_marker_.lifetime = rclcpp::Duration(0, 500000000);
	ball_marker_.scale.x = ball_marker_.scale.y = ball_marker_.scale.z = 0.204;
	ball_marker_.color.a = 1.0;
	ball_marker_.color.r = 1.0;
	ball_marker_.color.g = 1.0;

	last_time_ = this->now();
	lost_selfcheck_time_thres = selfcheck_time_thres;
	initialing_ = false;

	RCLCPP_INFO(this->get_logger(), "tracker_node初始化完成");
}

/*****************************************************************
 * @brief 球检测回调：坐标变换→跟踪器更新→发布结果+可视化
 *****************************************************************/
void TrackerNode::ball_callback(const volleyball_interfaces::msg::Ball::SharedPtr msg)
{
	if (!kf_initialized_ || initialing_) return;

	auto start = this->now();
	auto now = rclcpp::Time(msg->header.stamp);
	double dt = (now - last_time_).seconds();
	last_time_ = now;

	mm_ctx->pos_in_camera = Eigen::Vector3d(msg->x, msg->y, msg->z);

	try
	{
		auto tf = tf2_buffer_->lookupTransform(world_frame_, msg->header.frame_id, tf2::TimePointZero);
		Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y,
							 tf.transform.rotation.z);
		mm_ctx->camera2odom_rot = q.toRotationMatrix();
	}
	catch (const tf2::TransformException &ex)
	{
		RCLCPP_WARN(this->get_logger(), "TF转移矩阵获取失败: %s", ex.what());
		return;
	}

	// TF2 坐标变换：相机 → 底座坐标系
	geometry_msgs::msg::PointStamped ps;
	ps.header = msg->header;
	ps.header.stamp = now;
	ps.point.x = msg->x;
	ps.point.y = msg->y;
	ps.point.z = msg->z;

	Eigen::Vector3d world_pos;
	try
	{
		auto point = tf2_buffer_->transform(ps, world_frame_).point;
		world_pos << point.x, point.y, point.z;
	}
	catch (const tf2::TransformException &ex)
	{
		RCLCPP_WARN(this->get_logger(), "TF变换失败: %s", ex.what());
		return;
	}

	auto trans_msg = std::make_shared<volleyball_interfaces::msg::Ball>(*msg);
	trans_msg->x = world_pos(0);
	trans_msg->y = world_pos(1);
	trans_msg->z = world_pos(2);

	tracker_->update(trans_msg, dt);

	publish_tracked_ball(now);

	auto end = this->now();
	auto time = end - start;
	RCLCPP_DEBUG(this->get_logger(), "追踪耗费时间：%fs", time.seconds());
}

/*****************************************************************
 * @brief 丢检自检定时回调：超阈值且非 IDLE 时 Ekf 外推并发布
 *****************************************************************/
void TrackerNode::ball_lost_selfcheck_timer_callback()
{
	auto now = this->now();
	double dt = (now - last_time_).seconds();
	if (dt > lost_selfcheck_time_thres && tracker_->get_state() != Tracker::STATE_IDLE)
	{
		tracker_->predict_only(dt);
		last_time_ = now;
		if (tracker_->get_state() != Tracker::STATE_IDLE)
		{
			publish_tracked_ball(now);
		}
	}
}

/*****************************************************************
 * @brief 跟踪器复位服务回调
 *****************************************************************/
void TrackerNode::reset_tracker_callback(const std_srvs::srv::Trigger::Request::SharedPtr,
										 std_srvs::srv::Trigger::Response::SharedPtr response)
{
	tracker_->reset();
	response->success = true;
	RCLCPP_INFO(this->get_logger(), "tracker已复位!");
}

/*****************************************************************
 * @brief 发布跟踪结果及可视化标记
 *****************************************************************/
void TrackerNode::publish_tracked_ball(const rclcpp::Time &stamp)
{
	auto pos = tracker_->get_ball_position();

	// 发布 /tracker/target
	{
		volleyball_interfaces::msg::Ball out;
		out.header.stamp = stamp;
		out.header.frame_id = world_frame_;
		out.x = pos(0);
		out.vx = pos(1);
		out.y = pos(2);
		out.vy = pos(3);
		out.z = pos(4);
		out.vz = pos(5);
		track_pub_->publish(out);
		RCLCPP_DEBUG(this->get_logger(), "tracker_node发布滤波X=(%.1f,%.1f,%.1f),V=(%.1f,%1.f,%1.f)", out.x, out.y,
					 out.z, out.vx, out.vy, out.vz);
	}

	// 可视化
	{
		position_marker_.header.frame_id = world_frame_;
		position_marker_.header.stamp = stamp;
		position_marker_.pose.position.x = pos(0);
		position_marker_.pose.position.y = pos(2);
		position_marker_.pose.position.z = pos(4);

		linear_v_marker_.header = position_marker_.header;
		linear_v_marker_.points.clear();
		linear_v_marker_.points.emplace_back(position_marker_.pose.position);
		geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
		const double arrow_scale = 0.2;
		arrow_end.x += pos(1) * arrow_scale;
		arrow_end.y += pos(3) * arrow_scale;
		arrow_end.z += pos(5) * arrow_scale;
		linear_v_marker_.points.emplace_back(arrow_end);

		ball_marker_.header = position_marker_.header;
		ball_marker_.pose.position = position_marker_.pose.position;

		visualization_msgs::msg::MarkerArray marker_array;
		marker_array.markers.emplace_back(position_marker_);
		marker_array.markers.emplace_back(linear_v_marker_);
		marker_array.markers.emplace_back(ball_marker_);
		marker_pub_->publish(marker_array);
	}
}

}  // namespace volleyball