# volleyball_track

##
排球跟踪包，作为流水线的第二环。订阅检测结果，经 TF 变换到世界坐标系后输入扩展卡尔曼滤波器（EKF），输出平滑的排球位置与速度。包内提供观测模型、过程模型和卡尔曼滤波器内核解耦的头文件和工厂函数。最终只使用了**一次线形阻力模型**和**EKF~~不过已经退化为KF~~**对排球进行状态估计。


## ROS 接口

节点名：`tracker_node`

### Subscription

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 获取彩色相机焦距 `fx/fy`（用于测量噪声） |
| `/detector/ball` | `volleyball_interfaces/msg/Ball` | 排球检测结果（相机系 3D 位置） |

> 另通过 `tf2_ros::TransformListener` 监听 `/tf`、`/tf_static`，用于相机系→世界系坐标变换。

### Publisher

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/tracker/target` | `volleyball_interfaces/msg/Ball` | 滤波后位置与速度（`x/y/z` + `vx/vy/vz`，世界系） |
| `/tracker/marker` | `visualization_msgs/msg/MarkerArray` | 位置/速度矢量/球体可视化 |

### Service

| 服务 | 消息类型 | 说明 |
|------|----------|------|
| `/tracker/reset` | `std_srvs/srv/Trigger` | 复位跟踪器至 `IDLE` |

### Parameter

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `k` | double | `0.20` | 空气阻力系数 |
| `m` | double | `0.27` | 排球质量（kg） |
| `g` | double | `9.80` | 重力加速度 |
| `Q_sigma_xy_min` | double | `10.0` | 过程噪声 xy 方向下限（低速） |
| `Q_sigma_xy_max` | double | `300.0` | 过程噪声 xy 方向上限（高速） |
| `Q_sigma_z_min` | double | `15.0` | 过程噪声 z 方向下限 |
| `Q_sigma_z_max` | double | `400.0` | 过程噪声 z 方向上限 |
| `sigma_pixel` | int | `2` | 像素噪声标准差 |
| `sigma_depth_gain` | double | `0.04` | 深度噪声随距离增益 |
| `sigma_depth_const` | double | `0.05` | 深度噪声常数项 |
| `detect_cnt_thres` | int | `3` | 检测确认次数阈值 |
| `lost_time_thres` | double | `0.5` | 丢检超时阈值（s） |
| `selfcheck_time_thres` | double | `0.05` | 丢检自检定时周期（s） |
| `target_frame_id` | string | `odom` | 世界坐标系名称 |

## 原理

### 坐标变换

这里需要说明一下，球得到的坐标为相机光心坐标系(x向右，y向下，z向前)

### 状态估计（EKF）

- （待补充）状态量与状态方程：`[px, vx, py, vy, pz, vz]^T`。
- （待补充）过程模型：一次线性阻力 + 重力。
- （待补充）测量模型：位置观测。
- （待补充）过程噪声自适应（速度相关）。
- （待补充）测量噪声（像素噪声 + 深度噪声）。

### 状态机

- （待补充）`IDLE / DETECTING / TRACKING / TEMP_LOST` 各状态的迁移条件与作用。
