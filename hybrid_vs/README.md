# VolleyballRobot
Robocon2026 排球机器人上位机算法方案2：视觉伺服

## 1.项目简介与效果

---

## 2.架构



## 3.ROS2包说明
| 包 | 输入 | 输出 | 功能 |
|------|------|------|------|
| **volleyball_detect** | RealSense RGB-D | `/detector/ball` | YOLOv11n + OpenVINO 检测排球 3D 位置 |
| **volleyball_interfaces** | --- | --- | ROS2自定义消息接口 |
| **volleyball_robot** | --- | --- | 总启动包，参数文件存放于此 |
| **volleyball_robot_description** | --- | --- | 排球机器人机械描述 |
| **volleyball_serial_driver** | `/planner/plan` | UART 下位机 | 串口协议通信，发布里程计 TF |

## 4.项目目录


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
- detect 为上位机算法首环，将深度相机的RGB图像流与深度图像流同步，在RGB图像中找到排球2D图像坐标后，通过内外参得到排球2D坐标在深度图中的坐标，随后读取深度值估算出排球位置。  

## 7.待改进的地方


