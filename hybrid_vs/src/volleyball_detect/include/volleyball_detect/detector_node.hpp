/*******************************************************************************
 * @file detector_node.hpp
 * @brief 检测节点类，订阅RGBD图像并发布排球检测结果
 *******************************************************************************/

#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "image_transport/image_transport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// eigen
#include <eigen3/Eigen/Dense>
// OpenCV
#include <opencv2/opencv.hpp>
// realsense
#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "realsense2_camera_msgs/msg/extrinsics.hpp"
#include <librealsense2/rs.hpp>
// Project
#include "volleyball_detect/detector.hpp"
#include "volleyball_interfaces/msg/ball.hpp"
#include "volleyball_interfaces/msg/pid_camera.hpp"
// std
#include <mutex>

namespace volleyball
{

/****************************************************************
 * @class DetectorNode 检测排球节点类
 ****************************************************************/
class DetectorNode : public rclcpp::Node
{
private:
    // ---- 相机描述话题订阅（启动时等待参数到达） ----
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
    rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_sub_;
    // 缓存收到的相机参数
    sensor_msgs::msg::CameraInfo color_info_;
    sensor_msgs::msg::CameraInfo depth_info_;
    realsense2_camera_msgs::msg::Extrinsics extrinsics_;
    float ppx_;
    float ppy_;
    bool color_info_received_ = false;
    bool depth_info_received_ = false;
    bool extrinsics_received_ = false;
    std::mutex init_mutex_;
    bool initialized_ = false;
    void tryInitialize();  // 检查三个参数到齐后完成初始化

    // ---- 图像订阅（RGB深度图同步） ----
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_img_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_img_sub_;
    using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    // 球检测结果发布
    rclcpp::Publisher<volleyball_interfaces::msg::Ball>::SharedPtr ball_pub_;
    rclcpp::Publisher<volleyball_interfaces::msg::PidCamera>::SharedPtr pid_camera_pub_;
    // USB 相机订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr usb_cam_sub_;
    std::string camera_frame_id_;
    // 检测器实例
    std::shared_ptr<Detector> detector_;

    // ---- 调试发布 ----
    // 调试图像发布
    image_transport::Publisher detect_result_pub_;
    image_transport::Publisher colormap_depth_pub_;
    // Mark标记
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker ball_marker_;
    // 帧率统计
    int frame_cnt_;
    rclcpp::Time last_time_;
    // 调试帧跳过
    int debug_frame_skip_;
    int debug_frame_counter_;

    void RGBDImgCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,  // RGBD图像回调
                        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
    float getFPS();                                              // 计算帧率
    void createDebugPublisher();                                 // 创建调试图像发布器
    
public:
    DetectorNode();  // 构造函数
    ~DetectorNode() {}
    bool debug_;  // 调试模式标志
};

}  // namespace volleyball

#endif  // DETECTOR_NODE_HPP