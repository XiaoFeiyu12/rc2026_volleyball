/*******************************************************************************
 * @file detector_node.cpp
 * @brief 检测节点实现，订阅RGBD图像、调用检测器并发布结果
 *******************************************************************************/

#include "volleyball_detect/detector_node.hpp"

namespace volleyball
{
/*****************************************************************
 * @brief 构造函数：加载参数、初始化检测器、创建订阅与发布
 *****************************************************************/
DetectorNode::DetectorNode() : Node("detector_node"), frame_cnt_(0)
{
	/*--------------------------------------------------------------------------------------------------*/
	// 订阅相机描述话题，等待参数到齐后自动初始化
	auto cam_info_qos = rclcpp::QoS(1).reliable();
	auto extrinsics_qos = rclcpp::QoS(1).reliable().transient_local();

	color_info_sub_ =
		this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera/color/camera_info", cam_info_qos,
																[this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
																{
																	color_info_ = *msg;
																	color_info_received_ = true;
																	try_initialize();
																});

	depth_info_sub_ =
		this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera/depth/camera_info", cam_info_qos,
																[this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
																{
																	depth_info_ = *msg;
																	depth_info_received_ = true;
																	try_initialize();
																});

	extrinsics_sub_ = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
		"/camera/camera/extrinsics/depth_to_color", extrinsics_qos,
		[this](realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg)
		{
			extrinsics_ = *msg;
			extrinsics_received_ = true;
			try_initialize();
		});

	RCLCPP_INFO(this->get_logger(), "等待相机参数话题...");
}

/*****************************************************************
 * @brief 当三个相机参数话题全部到达后，初始化检测器和订阅发布
 *****************************************************************/
void DetectorNode::try_initialize()
{
	std::lock_guard<std::mutex> lock(init_mutex_);
	if (initialized_) return;
	if (!(color_info_received_ && depth_info_received_ && extrinsics_received_)) return;

	RCLCPP_INFO(this->get_logger(), "相机参数全部收到，开始初始化...");

	// ---- 从 CameraInfo 消息构建内参 ----
	// color k[row-major]: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
	cv::Mat color_camera_matrix = (cv::Mat_<float>(3, 3) << static_cast<float>(color_info_.k[0]),
								  static_cast<float>(color_info_.k[1]), static_cast<float>(color_info_.k[2]),
								  static_cast<float>(color_info_.k[3]), static_cast<float>(color_info_.k[4]),
								  static_cast<float>(color_info_.k[5]), static_cast<float>(color_info_.k[6]),
								  static_cast<float>(color_info_.k[7]), static_cast<float>(color_info_.k[8]));
	cv::Mat color_dist_coeffs = (cv::Mat_<float>(1, 5) << static_cast<float>(color_info_.d[0]),
								static_cast<float>(color_info_.d[1]), static_cast<float>(color_info_.d[2]),
								static_cast<float>(color_info_.d[3]), static_cast<float>(color_info_.d[4]));

	rs2_intrinsics color_camera_intrin;
	rs2_intrinsics depth_camera_intrin;
	rs2_extrinsics depth_to_color_extrin;
	rs2_extrinsics color_to_depth_extrin;

	color_camera_intrin.fx = static_cast<float>(color_info_.k[0]);
	color_camera_intrin.fy = static_cast<float>(color_info_.k[4]);
	color_camera_intrin.ppx = ppx_ = static_cast<float>(color_info_.k[2]);
	color_camera_intrin.ppy = ppy_ = static_cast<float>(color_info_.k[5]);
	color_camera_intrin.model = RS2_DISTORTION_BROWN_CONRADY;
	for (size_t i = 0; i < 5; i++) color_camera_intrin.coeffs[i] = static_cast<float>(color_info_.d[i]);

	depth_camera_intrin.fx = static_cast<float>(depth_info_.k[0]);
	depth_camera_intrin.fy = static_cast<float>(depth_info_.k[4]);
	depth_camera_intrin.ppx = static_cast<float>(depth_info_.k[2]);
	depth_camera_intrin.ppy = static_cast<float>(depth_info_.k[5]);
	depth_camera_intrin.model = RS2_DISTORTION_BROWN_CONRADY;
	for (size_t i = 0; i < 5; i++) depth_camera_intrin.coeffs[i] = static_cast<float>(depth_info_.d[i]);

	// ---- 从 Extrinsics 消息构建外参（rotation 为 column-major） ----
	Eigen::Map<const Eigen::Matrix3d> rot_map(extrinsics_.rotation.data());
	Eigen::Matrix3f depth_to_color_rotation = rot_map.cast<float>();

	Eigen::Matrix3f color_to_depth_rotation = depth_to_color_rotation.transpose();

	Eigen::Vector3f depth_to_color_translation(static_cast<float>(extrinsics_.translation[0]),
											   static_cast<float>(extrinsics_.translation[1]),
											   static_cast<float>(extrinsics_.translation[2]));
	Eigen::Vector3f color_to_depth_translation = -color_to_depth_rotation * depth_to_color_translation;

	// 填充 rs2_extrinsics (row-major)
	for (int row = 0; row < 3; row++)
		for (int col = 0; col < 3; col++)
			depth_to_color_extrin.rotation[row * 3 + col] = depth_to_color_rotation(row, col);

	depth_to_color_extrin.translation[0] = depth_to_color_translation.x();
	depth_to_color_extrin.translation[1] = depth_to_color_translation.y();
	depth_to_color_extrin.translation[2] = depth_to_color_translation.z();

	for (int row = 0; row < 3; row++)
		for (int col = 0; col < 3; col++)
			color_to_depth_extrin.rotation[row * 3 + col] = color_to_depth_rotation(row, col);

	color_to_depth_extrin.translation[0] = color_to_depth_translation.x();
	color_to_depth_extrin.translation[1] = color_to_depth_translation.y();
	color_to_depth_extrin.translation[2] = color_to_depth_translation.z();

	/*--------------------------------------------------------------------------------------------------*/
	// 模型文件获取
	std::string package_share_dir;
	try
	{
		package_share_dir = ament_index_cpp::get_package_share_directory("volleyball_detect");
	}
	catch (const std::exception &e)
	{
		RCLCPP_FATAL(this->get_logger(), "Failed to get package share directory: %s", e.what());
		throw;
	}
	// 声明并读取参数（使用默认值，相对路径）
	this->declare_parameter<std::string>("model_path", "model/volleyball_yolov11n.xml");
	std::string param_model_path;
	this->get_parameter("model_path", param_model_path);
	// 如果是相对路径（不以 '/' 开头），则拼接包共享目录
	if (param_model_path.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "Model path is empty!");
		throw std::runtime_error("Model path empty");
	}
	if (param_model_path.front() != '/')
	{
		param_model_path = package_share_dir + "/" + param_model_path;
	}
	RCLCPP_INFO(this->get_logger(), "Loading model from: %s", param_model_path.c_str());
	// 模型参数
	float model_confidence_threshold, model_nms_threshold, depth_validation_threshold;
	this->declare_parameter<float>("model_confidence_threshold", 0.7);
	this->declare_parameter<float>("model_nms_threshold", 0.5);
	this->declare_parameter<float>("depth_validation_threshold", 0.7);
	this->get_parameter("model_confidence_threshold", model_confidence_threshold);
	this->get_parameter("model_nms_threshold", model_nms_threshold);
	this->get_parameter("depth_validation_threshold", depth_validation_threshold);

	this->declare_parameter<bool>("debug", true);
	this->get_parameter("debug", debug_);
	this->declare_parameter<int>("debug_frame_skip", 5);
	this->get_parameter("debug_frame_skip", debug_frame_skip_);
	debug_frame_counter_ = 0;
	this->declare_parameter<std::string>("camera_frame_id", "camera_link");
	this->get_parameter("camera_frame_id", camera_frame_id_);

	/*--------------------------------------------------------------------------------------------------*/

	rgb_img_sub_ =
		std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/camera/color/image_raw");
	depth_img_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this,
																							"/camera/camera/depth/"
																							"image_rect_raw");
	sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(3), *rgb_img_sub_, *depth_img_sub_);
	sync_->registerCallback(
		std::bind(&DetectorNode::rgbd_img_callback, this, std::placeholders::_1, std::placeholders::_2));
	ball_pub_ = this->create_publisher<volleyball_interfaces::msg::Ball>("/detector/ball", rclcpp::SensorDataQoS());
	if (debug_)
	{
		create_debug_publisher();
	}

	detector_ = std::make_shared<Detector>(this->get_logger(), param_model_path, cv::Size(640, 640),
										   model_confidence_threshold, model_nms_threshold, depth_validation_threshold,
										   color_camera_matrix, color_dist_coeffs, color_camera_intrin,
										   depth_camera_intrin, depth_to_color_extrin, color_to_depth_extrin);
	last_time_ = this->now();

	initialized_ = true;
	RCLCPP_INFO(this->get_logger(), "detector_node初始化完成");
}

/*****************************************************************
 * @brief RGBD图像回调
 *****************************************************************/
void DetectorNode::rgbd_img_callback(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
								   const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg)
{
	try
	{
		auto start = this->now();
		auto rgb_img_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");
		auto depth_img_ptr = cv_bridge::toCvShare(depth_msg, "16UC1");
		const cv::Mat &rgb_img = rgb_img_ptr->image;
		const cv::Mat &depth_img = depth_img_ptr->image;
		frame_cnt_++;

		volleyball_interfaces::msg::Ball ball_msg;
		ball_msg.header.stamp = rgb_msg->header.stamp;

		detector_->detect(rgb_img, depth_img);

		if (!detector_->ball_list_.empty())
		{
			Ball volleyball = detector_->ball_list_.front();
			ball_msg.header.frame_id = camera_frame_id_;
			ball_msg.x = volleyball.x_;
			ball_msg.y = volleyball.y_;
			ball_msg.z = volleyball.z_;
			ball_msg.radius = volleyball.radius_3d_;
			ball_pub_->publish(ball_msg);
		}
		auto end = this->now();
		auto time = end - start;
		auto fps = get_fps();
		RCLCPP_DEBUG(this->get_logger(), "detector当前帧总耗时：%fms,帧率:%.1f", time.seconds() * 1000.0, fps);
		// Debug 图像：每隔 debug_frame_skip_ 帧发一张（降低开销）
		if (debug_ && (debug_frame_counter_++ % debug_frame_skip_ == 0))
		{
			auto rgb_show = rgb_img.clone();
			detector_->draw_detect_result(rgb_show, detector_->detection_box_list_);
			cv::putText(rgb_show, "FPS:" + std::to_string(static_cast<int>(fps)), cv::Point(16, 32),
						cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255));
			// 标记图像中心（绿色）和检测框中心（红色）及连线
			cv::circle(rgb_show, cv::Point(ppx_, ppy_), 5, cv::Scalar(0, 255, 0), 2);
			if (!detector_->detection_box_list_.empty())
			{
				const auto &best_box = *std::max_element(
					detector_->detection_box_list_.begin(), detector_->detection_box_list_.end(),
					[](const DetectionBox &a, const DetectionBox &b) { return a.confidence_ < b.confidence_; });
				cv::circle(rgb_show, cv::Point(static_cast<int>(best_box.cx_), static_cast<int>(best_box.cy_)), 5,
						   cv::Scalar(0, 0, 255), 2);
				cv::line(rgb_show, cv::Point(ppx_, ppy_),
						 cv::Point(static_cast<int>(best_box.cx_), static_cast<int>(best_box.cy_)), cv::Scalar(255, 0, 0),
						 2);
			}
			sensor_msgs::msg::Image::SharedPtr detection_result_msg =
				cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_show).toImageMsg();
			detect_result_pub_.publish(detection_result_msg);
		}
	}
	catch (const cv_bridge::Exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}
}

/*****************************************************************
 * @brief 计算并返回当前帧率
 *****************************************************************/
float DetectorNode::get_fps()
{
	float fps, dt;
	rclcpp::Time current_time = this->now();
	dt = (current_time - last_time_).seconds();
	fps = frame_cnt_ / dt;
	frame_cnt_ = 0;
	last_time_ = current_time;
	return fps;
}

/*****************************************************************
 * @brief 创建调试图像发布器（检测结果+深度图）
 *****************************************************************/
void DetectorNode::create_debug_publisher()
{
	detect_result_pub_ = image_transport::create_publisher(this, "detector/detection_result");
	colormap_depth_pub_ = image_transport::create_publisher(this, "detector/colormap_depth");

	marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detector/marker", 10);
	position_marker_.ns = "position";
	position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
	position_marker_.action = visualization_msgs::msg::Marker::ADD;
	position_marker_.lifetime = rclcpp::Duration(6, 0);
	position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
	position_marker_.color.a = 1.0;
	position_marker_.color.b = 1.0;

	ball_marker_.ns = "ball";
	ball_marker_.type = visualization_msgs::msg::Marker::SPHERE;
	ball_marker_.action = visualization_msgs::msg::Marker::ADD;
	ball_marker_.lifetime = rclcpp::Duration(0, 500000000);
	ball_marker_.scale.x = ball_marker_.scale.y = ball_marker_.scale.z = 0.204;
	ball_marker_.color.a = 1.0;
	ball_marker_.color.b = 1.0;
	ball_marker_.color.g = 1.0;
}

}  // namespace volleyball
