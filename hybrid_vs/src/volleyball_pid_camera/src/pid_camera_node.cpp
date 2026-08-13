#include "volleyball_pid_camera/pid_camera_node.hpp"

namespace volleyball
{
/*****************************************************************
 * @brief PidCameraNode 构造函数
 *****************************************************************/
PidCameraNode::PidCameraNode() : Node("pid_camera_node")
{
    // 获取 volleyball_detect 包共享目录（模型文件所在）
    std::string package_share_dir;
    try
    {
        package_share_dir = ament_index_cpp::get_package_share_directory("volleyball_detect");
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to get package share directory: %s", e.what());
        throw;
    }

    // 声明并读取参数
    this->declare_parameter<std::string>("model_path", "model/volleyball_yolov11n_int8/best.xml");
    std::string param_model_path;
    this->get_parameter("model_path", param_model_path);
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

    // 模型阈值
    float model_confidence_threshold, model_NMS_threshold;
    this->declare_parameter<float>("confidence_threshold", 0.7);
    this->declare_parameter<float>("NMS_threshold", 0.5);
    this->get_parameter("confidence_threshold", model_confidence_threshold);
    this->get_parameter("NMS_threshold", model_NMS_threshold);

    // 调试开关
    this->declare_parameter<bool>("debug", false);
    this->get_parameter("debug", debug_);
    this->declare_parameter<int>("debug_frame_skip", 5);
    this->get_parameter("debug_frame_skip", debug_frame_skip_);
    debug_frame_counter_ = 0;

    // USB 相机话题
    this->declare_parameter<std::string>("usb_cam_topic", "/usb_cam/image_raw");
    std::string usb_cam_topic;
    this->get_parameter("usb_cam_topic", usb_cam_topic);

    detector_ = std::make_shared<PidCameraDetector>(this->get_logger(), param_model_path, cv::Size(640, 640),
                         model_confidence_threshold, model_NMS_threshold);

    // 发布 /pid_camera
    pid_camera_pub_ = this->create_publisher<volleyball_interfaces::msg::PidCamera>(
        "/pid_camera", rclcpp::SensorDataQoS());

    // 订阅 USB 相机
    usb_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        usb_cam_topic, rclcpp::SensorDataQoS(),
        std::bind(&PidCameraNode::usbCamCallback, this, std::placeholders::_1));

    if (debug_)
    {
        debug_pub_ = image_transport::create_publisher(this, "pid_camera/debug");
    }

    RCLCPP_INFO(this->get_logger(), "pid_camera_node 初始化完成，订阅话题: %s", usb_cam_topic.c_str());
}

/*****************************************************************
 * @brief USB 相机图像回调 — 检测排球并发布像素偏移
 *****************************************************************/
void PidCameraNode::usbCamCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg)
{
    cv::Mat rgb_img = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
    try
    {
        int img_w = rgb_img.cols;
        int img_h = rgb_img.rows;

        // 纯彩色推理（跳过深度定位）
        detector_->detect(rgb_img);

        volleyball_interfaces::msg::PidCamera pid_msg;
        if (detector_->detection_box_list_.empty())
        {
            // 无检测框 → 零偏移
            pid_msg.pixel_diff_x = 0.0f;
            pid_msg.pixel_diff_y = 0.0f;
        }
        else
        {
            // 取置信度最高的检测框
            const auto& best_box = *std::max_element(
                detector_->detection_box_list_.begin(), detector_->detection_box_list_.end(),
                [](const DetectionBox& a, const DetectionBox& b) { return a.confidence < b.confidence; });
            float box_cx = best_box.cx;
            float box_cy = best_box.cy;
            double scale_x = img_w / best_box.box.width;
            double scale_y = img_h / best_box.box.height;
            pid_msg.pixel_diff_x = (box_cx - static_cast<float>(img_w) / 2.0f - 130) * scale_x;
            pid_msg.pixel_diff_y = - (box_cy - static_cast<float>(img_h) / 2.0f + 80) * scale_y;

            RCLCPP_DEBUG(this->get_logger(),"dx:%.1f, dy:%.1f", pid_msg.pixel_diff_x, pid_msg.pixel_diff_y);
        }

        pid_camera_pub_->publish(pid_msg);

        // Debug 图像：每隔 debug_frame_skip_ 帧发一张（降低开销）
        if (debug_ && (debug_frame_counter_++ % debug_frame_skip_ == 0))
        {
            detector_->drawDetectResult(rgb_img, detector_->detection_box_list_);
            // 标记图像中心（绿色）和检测框中心（红色）及连线
            cv::circle(rgb_img, cv::Point(img_w / 2 +130, img_h / 2 -80), 5, cv::Scalar(0, 255, 0), 2);
            if (!detector_->detection_box_list_.empty())
            {
                const auto& best_box = *std::max_element(
                    detector_->detection_box_list_.begin(), detector_->detection_box_list_.end(),
                    [](const DetectionBox& a, const DetectionBox& b) { return a.confidence < b.confidence; });
                cv::circle(rgb_img, cv::Point(static_cast<int>(best_box.cx ), static_cast<int>(best_box.cy)),
                        5, cv::Scalar(0, 0, 255), 2);
                cv::line(rgb_img, cv::Point(img_w / 2 +130, img_h / 2 -80),
                        cv::Point(static_cast<int>(best_box.cx), static_cast<int>(best_box.cy)),
                        cv::Scalar(255, 0, 0), 1);
            }
            sensor_msgs::msg::Image::SharedPtr debug_msg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_img).toImageMsg();
            debug_pub_.publish(debug_msg);
        }
    }
    catch(const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    

}

}  // namespace volleyball
