# VolleyballRobot
Robocon2026 排球机器人上位机算法方案2：视觉伺服

## 1.项目简介与效果
这个方案是在一篇论文[1]的启发下得到的。众所周知，在传感器精度不高、算法实现困难的情况下，通过纯视觉的闭环控制，将目标的2D像素坐标差输入PID控制，可能横向方向响应会比较快收敛跟随球左右移动，而单凭一个彩色图像缺乏深度，纵向上对于一个抛物运动的球难以前后响应。那么如果将球的三维坐标输入呢？  
经过实验发现，将球的纵向深度作为底盘的纵向速度控制的输入量，将球的横向坐标作为底盘横向速度控制输入量，小车能够先快速响应向手中的排球冲过来，随后逐渐减速停在球正前方。但可惜的是，由于时间有限，以及这是个临时方案，仅用PID会出现当球与车速度方向相反时，例如球摄像头前从左向右快速飞过，车会先追赶然后发现球早已飞过的情况，并不能对球的运动趋势作出正确规划反映。

---

## 2.架构
``` mermaid
graph LR

A[目标检测]-->B[IBVS控制]-->C[串口通信]-->D[下位机]
```


## 3.ROS2包说明
| 包 | 输入 | 输出 | 功能 |
|------|------|------|------|
| **volleyball_detect** | RealSense RGB-D | `/detector/ball` | YOLOv11n + OpenVINO 检测排球 3D 位置 |
| **volleyball_interfaces** | --- | --- | ROS2自定义消息接口 |
| **volleyball_robot** | --- | --- | 总启动包，参数文件存放于此 |
| **volleyball_robot_description** | --- | --- | 排球机器人机械描述 |
| **volleyball_serial_driver** | `/planner/plan` | UART 下位机 | 串口协议通信 |

## 4.项目目录
ibvs  
├── src  
│   ├── volleyball_detect  // 目标检测模块  
│   ├── volleyball_ibvs  //视觉伺服  
│   ├── volleyball_interfaces //ROS2自定义接口  
│   ├── volleyball_robot // 上位机软件总启动包  
│   ├── volleyball_robot_description // 机器人描述包  
│   └── volleyball_serial_driver // 串口包  
└── tools // 工具脚本  
    ├── extract_img.py
    ├── record_dataset.py
    └── reload.sh

## 5.部署

### 本地部署

#### 本人使用项目环境
- 操作系统：Ubuntu 22.04
- ROS 2 Humble
- 相机：realsense d435
- 运算平台：minipc (CPU i5-12450)
- 通信方式：MicroUSB虚拟串口 type-c

#### 1. 拉取仓库
```bash
git clone https://github.com/XiaoFeiyu12/rc2026_volleyball.git
cd rc2026_volleyball/hybrid_vs
```

#### 2. 安装系统依赖
```bash
sudo apt-get update && sudo apt-get install -y \
    wget gpg gnupg lsb-release ca-certificates tzdata \
    python3 python3-pip python3-yaml \
    libeigen3-dev \
    libopencv-dev \
    ros-humble-rclcpp \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-message-filters \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-std-srvs \
    ros-humble-foxglove-bridge
```

#### 3. 安装 RealSense SDK
```bash
sudo apt-get update && sudo apt-get install -y \
    ros-humble-librealsense2* \
    ros-humble-realsense2-*
```

#### 4. 安装 OpenVINO
具体安装查询[OpenVINO官方文档](https://www.intel.cn/content/www/cn/zh/developer/tools/openvino-toolkit/download.html?PACKAGE=OPENVINO_BASE&VERSION=v_2026_3_0&OP_SYSTEM=LINUX&DISTRIBUTION=PIP)

#### 5. 串口权限
串口设备（默认 `/dev/ttyACM0`）需要 `dialout` 组权限才能访问：
```bash
sudo usermod -a -G dialout $USER
```
> **注意：** 执行后需要**注销重新登录**（或重启）才能生效。可以用 `groups $USER` 确认是否已加入 `dialout` 组。

#### 6. 构建
```bash
source /opt/ros/humble/setup.sh
source /opt/intel/openvino/setupvars.sh
colcon build --parallel-workers 4
```

#### 7. 运行
```bash
source install/setup.bash
ros2 launch volleyball_robot bringup.launch.py
```

#### 8. 修改代码后重新编译
```bash
# 在工作区根目录执行
source /opt/intel/openvino/setupvars.sh
colcon build --parallel-workers 4
# 然后 Ctrl+C 停止当前 launch，重新运行即可
```

### 设置开机自启动
```bash
# 1. 创建 systemd 服务文件
sudo nano /etc/systemd/system/volleyball-robot.service
```
写入以下内容（将 `YOUR_USER` 替换为你的用户名，将 `/YOUR_PATH` 替换为实际路径）：
```
[Unit]
Description=Volleyball robot autostart (local)
After=network.target

[Service]
Type=simple
User=YOUR_USER
WorkingDirectory=/YOUR_PATH
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.sh && source /opt/intel/openvino/setupvars.sh && source install/setup.bash && ros2 launch volleyball_robot bringup.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
```bash
# 2. 重新加载 systemd 配置
sudo systemctl daemon-reload

# 3. 设置开机自启
sudo systemctl enable volleyball-robot.service

# 4. 立即启动测试
sudo systemctl start volleyball-robot.service

# 5. 查看状态（确认 active 且无报错）
sudo systemctl status volleyball-robot.service

# 关闭自启动
sudo systemctl disable volleyball-robot.service
```

## 6.原理
这里不会具体展开，只进行大致讲解，具体可以看各个功能包下的README文件。  
- detect 为上位机算法首环，将深度相机的RGB图像流与深度图像流同步，在RGB图像中找到排球2D图像坐标后，通过内外参得到排球2D坐标在深度图中的坐标，随后读取深度值估算出排球位置。[detect包介绍](src/volleyball_detect/README.md)
- ibvs  这个包为底盘控制外环，根据球相对击球盘的三维空间位置控制底盘运动速度，且包含自动击打逻辑。[ibvs包介绍](src/volleyball_ibvs/README.md)

## 7.补充与改进点
说实话，该方案表现出来确实是击球盘能接触到球的机会更多了，但目前简陋的算法逻辑并不能支持他预判球的飞行方向去真正接到球。改进思路有以下几种：  
1. 弥补画面采集到通信下发这段控制环节中的纯滞后环节，通过变种卡尔曼滤波等方式。
2. 参照论文，加入机器人自身状态估计。

## 8.参考文献
[1] K. Yang, C. Bai, Z. She and Q. Quan, "High-Speed Interception Multicopter Control by Image-Based Visual Servoing," in IEEE Transactions on Control Systems Technology, vol. 33, no. 1, pp. 119-135, Jan. 2025, doi: 10.1109/TCST.2024.3451293.
keywords: {Drones;Cameras;Vectors;Delays;Observers;Visualization;Accuracy;Anti-drone system;delayed Kalman filter (DKF);high-speed interception;image-based visual servoing (IBVS)},



