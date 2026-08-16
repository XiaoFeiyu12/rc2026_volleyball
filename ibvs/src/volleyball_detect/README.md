# volleyball_detect

## 简介
视觉检测包，作为上位机算法流水线的第一环。订阅 RGB 图像与深度图像，通过 YOLOv11n + OpenVINO 检测排球的 2D 位置，通过转换得到排球在深度图中的位置，再结合相机内外参估算排球的 3D 位置并发布。

## ROS 接口

节点名：`detector_node`

### Subscription

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | realsense 彩色相机内参 |
| `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | realsense 深度相机内参 |
| `/camera/camera/extrinsics/depth_to_color` | `realsense2_camera_msgs/msg/Extrinsics` | 深度到彩色的外参 |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | realsense 彩色图像流|
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | realsense 深度图像流|

### Publisher

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/detector/ball` | `volleyball_interfaces/msg/Ball` | 排球 3D 位置`x/y/z` |

调试发布（`debug` 参数开启时）：

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `detector/detection_result` | `sensor_msgs/msg/Image` | 检测框绘制图，含FPS |
| `detector/colormap_depth` | `sensor_msgs/msg/Image` | 深度伪彩色图 |
| `detector/marker` | `visualization_msgs/msg/MarkerArray` | 3D 位置可视化 |

### Parameter

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | string | `model/volleyball_yolov11n.xml` | 模型路径（写相对路径即可）|
| `model_confidence_threshold` | float | `0.7` | 检测置信度阈值 |
| `model_nms_threshold` | float | `0.5` | NMS 重叠阈值 |
| `depth_validation_threshold` | float | `0.7` | 深度法与几何法交叉验证的相对误差阈值 |
| `debug` | bool | `true` | 是否开启调试发布 |
| `debug_frame_skip` | int | `5` | 调试图像隔帧发布数 |
| `camera_frame_id` | string | `camera_link` | 发布球消息的坐标系 |

## 原理

### 图像流

以下是针对realsense d435而言发现的情况：
1. 如果开启`enable_sync`,`align_depth.enable`,`enable_rgbd`参数选项的话，realsense SDK会自动将深度图对齐至彩色图，输出RGB-D数据流，此时彩色图与深度图的位置是完全对应的，但缺点是由于计算量大加上同步时间戳会导致严重调帧、卡顿，尤其是画面出现运动的时候。
2. realsense d435的彩色图为卷帘快门而非全局快门，选型时注意选择。  

因此针对以上情况，我们退而求其次，仅开启硬件同步`enable_sync`，同时利用`message_filters`实现彩色图与深度图话题同步订阅。后续深度获取在后续局部进行图像对齐。

### 检测（YOLOv11n + OpenVINO）
> 注：检测模型基于 Ultralytics YOLO11n 训练，权重受 AGPL-3.0 许可约束，故本仓库不随附模型文件。请自行训练（单类别 `volleyball`）并导出为 OpenVINO IR（`best.xml` + `best.bin`）放入 `model/` 目录，或通过 `model_path` 参数指向你的模型。  
这里网上资料还是挺多的，包括官方部署API，比较固定。主要流程为下：

``` mermaid
graph LR

A[模型预加载]
B[RGB图像流] --> C[预处理] --> D[推理] --> E[后处理] --> F[筛选]
```
这里筛选考虑到一般只有一个排球直接通过，但如果出现多检测目标（含误检）只保留置信度最高的那一个。所以实际测试的时候发现往往会有误检的情况发生，而且不知道是不是模型训练有问题，有时候错误目标置信度比排球都大，在训练时需要多增加空白样本、负样本，尤其是黄、白、蓝色相关物品，比如顶灯（这个可能是我数据集以及训练问题）、广告牌（尤其是那些蓝白色、黄蓝色赞助商的图标）。   

这里由于考虑计算性能只使用了yolo11n检测模型 + 量化 + openvino CPU推理，如果条件允许可以试试用更好的模型，使用GPU推理。

### 3D 定位

在YOLO得到排球的检测框之后，经过实际测试，将整幅深度图对齐至彩色图延迟是很高的，所以我选择在YOLO框ROI内，以中心半径为9 pixel的方形区域（注意防出界）内将每一像素利用realsense官方提供API算法将彩色图坐标映射至深度图坐标，取深度图对应位置的深度$depth$，找到深度最小的点作为球表面离相机最近的点$A$，  

计算深度：  
$$
\begin{aligned}
    z= d_{min} \cdot depth    
\end{aligned}
$$   
得到坐标：  
$$
\begin{aligned}
    x=\frac{(x-c_x)z}{f_x}  \\
    y=\frac{(y-c_y)z}{f_y}   
\end{aligned}
$$   


以上坐标$(x,y,z)^T$为相机坐标系

随后可以得到相机光心到点$A$的方向向量
$$
\begin{aligned}
    \vec{r} = \frac{(x,y,z)}{\lVert (x,y,z) \rVert}
\end{aligned}
$$
最后设排球半径为$R$,球心位置可得：
$$
\begin{aligned}
    (x_b,y_b,z_b) = (x,y,z) + \vec{r} \cdot R
\end{aligned}
$$

当然整个过程需要剔除超出深度相机可用范围的或者缺陷深度为0的点，此外为避免以上情况以及深度图质量的情况，加入基本的几何法测距作为验证。由于检测框宽高往往不一致，故公式如下：

$$
\begin{aligned}
    r_w &= width_{\text{识别框}} /2\\
    r_h &= height_{\text{识别框}}  / 2 \\
    z_w &=  \sqrt{(\frac{f_x \cdot R}{r_w})^2 + R^2} \\
    z_h &=  \sqrt{(\frac{f_y \cdot R}{r_h})^2 + R^2} \\
    z &= \frac{z_w+z_h}{2} \\
    x&=\frac{(x-c_x)z}{f_x}  \\
    y&=\frac{(y-c_y)z}{f_y}  
\end{aligned}
$$

虽然精度终究是比通过深度度估算差，但作为下限兜底验证还是可以的。

## 补充与可改进之处
以上算法终究受限于相机本身精度4m内2%的误差上的，且受限于计算性能也不敢选择过于复杂的算法。  
如果你问为什么没有用PnP,原因是球他没有矩形明显角点，他损失了旋转的三个自由度，实际使用下来发现pnp算法乱飘。  
目前尚未找到更好的测距方法，唯一能想到的是通过改进设备，选择更优的双目相机组模块来改进这个问题。  
当然对于误识别的处理，现在回头看其实还是有欠考虑的有些生硬的，其实可以加入连续帧数判断的。

