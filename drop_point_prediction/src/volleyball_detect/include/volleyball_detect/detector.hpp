/*******************************************************************************
 * @file detector.hpp
 * @brief 排球检测器类，基于OpenVINO推理+YOLO模型实现目标检测与3D定位
 *******************************************************************************/

#ifndef DETECTOR_HPP
#define DETECTOR_HPP

// STL
#include <string>
#include <vector>
// OpenCV
#include <opencv2/opencv.hpp>
// OpenVINO
#include <openvino/openvino.hpp>
// Eigen
#include <eigen3/Eigen/Dense>
// realsense
#include <librealsense2/rs.hpp>
// ROS2
#include "rclcpp/rclcpp.hpp"

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
	cv::Rect box_;
	float confidence_;
	short class_id_;
	float cx_;
	float cy_;
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
	float x_;
	float y_;
	float z_;
	float radius_3d_ = 0.102;
	float confidence_ = 0.0;
	uint8_t position_type_ = 0;	// 0=depth , 1=geometry
} Ball;

/****************************************************************
 * @class Detector 检测器类
 ****************************************************************/
class Detector
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
	float NMS_threshold_;		  // NMS阈值
	// RealSense深度图参数
	float depth_scale_ = 0.001;	 // 深度比例尺（mm→m）
	static const int DEPTH_MAX = 7500;
	static const int DEPTH_MIN = 0;
	rs2_intrinsics color_camera_intrin_;	// 彩色相机内参
	rs2_intrinsics depth_camera_intrin_;	// 深度相机内参
	rs2_extrinsics depth_to_color_extrin_;	// 深度→彩色外参
	rs2_extrinsics color_to_depth_extrin_;	// 彩色→深度外参
	// 深度图or几何法
	float depth_validation_threshold_;

	// 图像预处理
	void pre_processing(const cv::Mat &img);
	// 模型输出后处理（解析检测框+NMS）
	void post_processing(std::vector<DetectionBox> &box_list);
	// 将检测框缩放回原图尺寸
	cv::Rect get_bounding_box(const cv::Rect &src) const;
	// 执行完整推理流程
	std::vector<DetectionBox> infer(const cv::Mat &input_img);

	// 获取排球3D位置（深度图法）
	bool get_ball_pos_depth_img(Ball &volleyball, const DetectionBox &detect_box, const cv::Mat &depth_img);
	// 获取排球3D位置（几何估算法）
	void get_ball_pos_geometry(Ball &volleyball, const DetectionBox &detect_box);

public:
	// 构造函数：加载模型并初始化推理引擎
	Detector(const rclcpp::Logger logger, const std::string &model_path, const cv::Size model_input_shape,
			 const float confidence_threshold, const float NMS_threshold, const float depth_validation_threshold,
			 const cv::Mat &color_cameraMatrix, const cv::Mat &color_distCoeffs,
			 const rs2_intrinsics &color_camera_intrin, const rs2_intrinsics &depth_camera_intrin,
			 const rs2_extrinsics &depth_to_color_extrin, const rs2_extrinsics &color_to_depth_extrin);
	~Detector() {};	 // 析构函数

	// 执行检测（彩色图+深度图）
	void detect(const cv::Mat &color_img, const cv::Mat &depth_img);
	// 绘制检测结果
	void draw_detect_result(cv::Mat &color_img, const std::vector<DetectionBox> &detect_box_list);

	std::vector<DetectionBox> detection_box_list_;	// 当前帧检测框列表
	std::vector<Ball> ball_list_;					// 当前帧排球3D位置列表
};

}  // namespace volleyball

#endif	// DETECTOR_HPP