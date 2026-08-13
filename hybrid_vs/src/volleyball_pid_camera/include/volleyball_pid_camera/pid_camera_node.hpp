#ifndef PID_CAMERA_NODE_HPP
#define PID_CAMERA_NODE_HPP

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
// opencv
#include <opencv2/opencv.hpp>
// Project
#include "volleyball_interfaces/msg/pid_camera.hpp"
#include "volleyball_pid_camera/pid_camera_detector.hpp"

namespace volleyball
{

/****************************************************************
 * @class PidCameraNode USB 相机 PID 视觉伺服节点
 * 订阅 USB 相机画面，使用 YOLO 模型检测排球，
 * 计算检测框中心与图像中心的像素偏移，发布到 /pid_camera 话题
 ****************************************************************/
class PidCameraNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr usb_cam_sub_;
    rclcpp::Publisher<volleyball_interfaces::msg::PidCamera>::SharedPtr pid_camera_pub_;

    std::shared_ptr<PidCameraDetector> detector_;

    bool debug_;
    int debug_frame_skip_;
    int debug_frame_counter_;
    std::vector<float> history_cx_{3};
    std::vector<float> history_cy_{3};
    image_transport::Publisher debug_pub_;
    void usbCamCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg);

public:
    PidCameraNode();
    ~PidCameraNode() {}
};

}  // namespace volleyball

#endif  // PID_CAMERA_NODE_HPP
