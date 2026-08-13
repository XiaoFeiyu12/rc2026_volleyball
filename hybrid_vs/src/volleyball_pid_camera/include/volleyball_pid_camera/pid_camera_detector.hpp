/*******************************************************************************
 * @file detector.hpp
 * @brief 排球检测器类，基于OpenVINO推理+YOLO模型实现目标检测与3D定位
 *******************************************************************************/

#ifndef DETECTOR_HPP
#define DETECTOR_HPP

// STL
#include <string>
#include <vector>
// OpenVINO
#include <openvino/openvino.hpp>
// Eigen
#include <eigen3/Eigen/Dense>
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/msg/image.hpp"
// OpenCV
#include <opencv2/opencv.hpp>


namespace volleyball
{
/*******************************************************************************
 * @struct DetectionBox
 * @brief 目标检测框结构体
 * @param box 检测框坐标
 * @param confidence 置信度
 * @param class_id 类别ID
 * @param cx 检测框中心x坐标
 * @param cy 检测框中心y坐标
 *******************************************************************************/
typedef struct _box_
{
    cv::Rect box;
    float confidence;
    short class_id;
    float cx;
    float cy;
} DetectionBox;

/*******************************************************************************
 * @struct Ball
 * @brief 排球3D位置结构体
 * @param x 排球x坐标
 * @param y 排球y坐标
 * @param z 排球z坐标
 * @param radius_3d 排球半径（默认0.102m）
 *******************************************************************************/
typedef struct _ball_
{
    float x;
    float y;
    float z;
    float radius_3d = 0.102;
    float confidence = 0.0;
    uint8_t position_type = 0; // 0=depth , 1=geometry
} Ball;

/****************************************************************
 * @class Detector 检测器类
 ****************************************************************/
class PidCameraDetector
{
private:
    // ROS2
    rclcpp::Logger logger_;

    // OpenVINO推理引擎
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    // 模型输入输出尺寸
    cv::Size2f model_input_shape_;
    cv::Size model_output_shape_;
    // letterbox参数
    float letterbox_scale_;
    float padding_x_;
    float padding_y_;
    // 检测阈值
    float confidence_threshold_;  // 置信度阈值
    float NMS_threshold_;        // NMS阈值

    // 图像预处理
    void PreProcessing(const cv::Mat& img);
    // 模型输出后处理（解析检测框+NMS）
    void PostProcessing(std::vector<DetectionBox>& box_list);
    // 将检测框缩放回原图尺寸
    cv::Rect GetBoundingBox(const cv::Rect& src) const;
    // 执行完整推理流程
    std::vector<DetectionBox> infer(const cv::Mat& input_img);

public:
    // 构造函数：加载模型并初始化推理引擎
    PidCameraDetector(const rclcpp::Logger logger, const std::string& model_path, const cv::Size model_input_shape,
             const float confidence_threshold, const float NMS_threshold);
    ~PidCameraDetector(){};  // 析构函数

    // 执行检测（彩色图+深度图）
    void detect(const cv::Mat& color_img);
    // 绘制检测结果
    void drawDetectResult(cv::Mat& color_img, const std::vector<DetectionBox>& detect_box_list);

    std::vector<DetectionBox> detection_box_list_;  // 当前帧检测框列表
    std::vector<Ball> ball_list_;                    // 当前帧排球3D位置列表
};

}  // namespace volleyball

#endif  // DETECTOR_HPP