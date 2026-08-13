# volleyball_detect — 排球视觉检测包

## 概述

`volleyball_detect` 是排球机器人的视觉感知核心。它从 **Intel RealSense** 相机获取 RGB-D 图像，利用 **OpenVINO** 推理框架运行 **YOLOv11** 目标检测模型，实时识别图像中的排球，并通过多种测距手段计算排球在相机坐标系下的 **3D 位置坐标**。

## 架构

```
RGB-D 相机 → 图像同步(message_filters) → 目标检测(OpenVINO) → 3D定位 → /detector/ball
```

## 节点

### `detector_node` (主节点)

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RGB 彩色图像 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图（16UC1） |

两个话题通过 `message_filters::ApproximateTime` 同步，确保 RGB 与深度图时间戳对齐。

#### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/detector/ball` | `volleyball_interfaces/msg/Ball` | 检测到的排球 3D 位置（相机坐标系） |
| `/detector/detection_result` | `sensor_msgs/Image` | (调试) 带检测框的 RGB 图像 |
| `/detector/colormap_depth` | `sensor_msgs/Image` | (调试) 伪彩色深度图 |

#### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model_path` | `model/volleyball_yolov11n.xml` | 相对/绝对路径的 OpenVINO 模型文件 |
| `model_confidence_threshold` | `0.7` | 检测置信度阈值 |
| `model_NMS_threshold` | `0.4` | NMS 非极大抑制阈值 |
| `debug` | `true` | 是否发布调试图像 |
| `camera_frame_id` | `camera_link` | 相机坐标系名称 |
| `depth_to_rgb.R` | — | 深度→彩色旋转矩阵 (3×3) |
| `depth_to_rgb.T` | — | 深度→彩色平移向量 (3×1) |
| `color_camera.K` | — | 彩色相机内参矩阵 (3×3) |
| `color_camera.D` | — | 彩色相机畸变系数 (1×5) |
| `depth_camera.K` | — | 深度相机内参矩阵 (3×3) |
| `depth_camera.D` | — | 深度相机畸变系数 (1×5) |

## 核心类

### `Detector` — 检测器算法类

位于 `include/volleyball_detect/detector.hpp`，封装了完整的检测与测距流程。

#### 图像预处理 (`PreProcessing`)
- 将输入图像缩放到模型输入尺寸（640×640）
- 使用 `OpenVINO PrePostProcessor` 完成 BGR→RGB 转换、归一化（除以 255）
- 将预处理后的数据写入推理请求输入张量

#### 模型推理 (`infer`)
- 调用 `infer_request_.infer()` 执行同步推理
- 测量并打印推理耗时（ms）

#### 后处理 (`PostProcessing`)
- 从输出张量提取各检测框的中心坐标、宽高、类别分数
- 取每个检测框最高分类别，过滤低于置信度阈值的结果
- 执行 **NMS（非极大值抑制）** 去除重叠框
- 将检测框坐标从模型输入尺寸缩放回原始图像尺寸

#### 三种测距方式

1. **`getBallPos_depth_img`（深度图法）** ⭐（默认使用）
   - 在检测框中心搜索半径内遍历所有像素
   - 使用 RealSense SDK 的 `rs2_project_color_pixel_to_depth_pixel` 将彩色像素映射到深度图
   - 取深度值最小的点，利用小孔成像模型反算 3D 坐标
   - 沿视线方向补偿球的半径（`surface_point + direction * radius`）
   - 将 OpenCV 坐标系（X右 Y下 Z前）转换为 ROS 相机坐标系（X前 Y左 Z上）

2. **`getBallPos_PnP`（PnP 法）**
   - 以球半径构建 5 个 3D 关键点（球心、四方向边缘）
   - 通过 3D-2D 对应点使用 `cv::solvePnP` 求解球心位置

3. **`getBallPos_geometry`（几何估算法）**
   - 利用球在图像中的像素直径和球的实际半径，根据相似三角形估算深度

## 数据结构

```cpp
struct DectectionBox {
    cv::Rect box;     // 像素坐标系下的检测框
    float confidence; // 置信度
    short class_id;   // 类别 ID
    float cx, cy;     // 检测框中心像素坐标
};

struct Ball {
    float x, y, z;    // 球在相机坐标系下的 3D 位置 (m)
    float radius_3d;  // 球的物理半径 (m)，默认 0.102
};
```

## Launch 文件

`volleyball_detect.launch.py` 同时启动：
1. **RealSense 相机驱动**（通过 `include` 方式引入 `realsense2_camera` 的 launch 文件）
2. **`detector_node`**（加载 `detect_cfg.yaml` 中的参数）

RealSense 相机参数从 `config/realsense_cfg.yaml` 读取。

## 配置文件

| 文件 | 说明 |
|------|------|
| `config/detect_cfg.yaml` | 模型参数、相机内外参、调试开关 |
| `config/realsense_cfg.yaml` | RealSense 相机驱动配置（分辨率、帧率等） |

## 模型文件

`model/` 目录存放 OpenVINO 格式的 YOLOv11 模型（`.xml` + `.bin`），默认使用 `volleyball_yolov11n_int8/best.xml`（INT8 量化版）。

## 依赖

- `rclcpp` — ROS2 C++ 客户端库
- `sensor_msgs` — 图像消息类型
- `vision_opencv` / `cv_bridge` — ROS ↔ OpenCV 图像转换
- `OpenVINO` — 神经网络推理引擎
- `librealsense2` — RealSense 相机 SDK
- `message_filters` — 时间同步
- `image_transport` — 图像传输
- `volleyball_interfaces` — 自定义消息
- `Eigen3` — 线性代数库

## 数据流

```
RealSense 相机
    ↓ RGB + Depth（同步）
detector_node (Detector)
    ↓ 目标检测 (YOLOv11 + OpenVINO)
    ↓ 3D 定位 (深度图对齐/PnP/几何估算)
/detector/ball  →  volleyball_track (追踪滤波)
```
