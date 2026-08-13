# VolleyballRobot
Robocon 排球机器人

## 项目简介

基于 ROS 2 Humble 的排球机器人系统，实现排球检测、追踪、轨迹预测与接球规划的全流程自动化。

---

## 系统架构

```
RealSense D435 → YOLOv11n 检测 → EKF 追踪 → RK4 轨迹预测 → 接球位姿规划 → 串口下发 → 下位机
```

| 模块 | 输入 | 输出 | 功能 |
|------|------|------|------|
| **volleyball_detect** | RealSense RGB-D | `/detector/ball` | YOLOv11n + OpenVINO 检测排球 3D 位置 |
| **volleyball_track** | `/detector/ball` | `/tracker/target` | 线性阻力模型 EKF，输出平滑位置和速度 |
| **volleyball_predict** | `/tracker/target` | `/predictor/ball_trajectory` | RK4 数值积分预测球轨迹 |
| **volleyball_plan** | `/predictor/ball_trajectory` | `/planner/plan` | 计算接球位姿 |
| **volleyball_serial_driver** | `/planner/plan` | UART 下位机 | 串口协议通信，发布里程计 TF |

---

## 部署

### 本地部署

#### 前置条件
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble（需提前安装，参考 [ROS 2 官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)）

#### 1. 拉取仓库
```bash
git clone https://gitee.com/liu-chengtao122/2026-robocon-volleyball.git
cd 2026-robocon-volleyball
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
```bash
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt update && sudo apt install -y openvino-2026.2.0
# 创建通用路径软链接（若 apt 安装后未自动创建）
sudo ln -sf /opt/intel/openvino_2024 /opt/intel/openvino
# 安装完成后加载 OpenVINO 环境
source /opt/intel/openvino/setupvars.sh
```

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


### Docker 部署
```bash
#首先拉取仓库
git clone https://gitee.com/liu-chengtao122/2026-robocon-volleyball.git
```

```bash
# 1. 构建镜像(这边最好开梯子拉镜像不然容易挂)
docker build --network host -t volleyball-robot:latest .

# 2. 启动
docker compose up

# 3. 修改代码后一键编译重启
. tools/reload.sh

# 4. 停止
docker compose down
```

### 设置开机自启动

#### 本地部署自启动
```bash
# 1. 创建 systemd 服务文件
sudo nano /etc/systemd/system/volleyball-robot.service
```
写入以下内容（将 `YOUR_USER` 替换为你的用户名，将 `/path/to/2026-robocon-volleyball` 替换为实际路径）：
```
[Unit]
Description=Volleyball robot autostart (local)
After=network.target

[Service]
Type=simple
User=YOUR_USER
WorkingDirectory=/path/to/2026-robocon-volleyball
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

#### Docker 部署自启动
```bash
# 先确保 Docker 能够自启动
sudo systemctl enable docker
```
```bash
# 1. 创建 systemd 服务文件
sudo nano /etc/systemd/system/volleyball-robot.service
```
写入以下内容（将 `YOUR_USER` 替换为你的用户名，将 `/path/to/2026-robocon-volleyball` 替换为实际路径）：
```
[Unit]
Description=Volleyball robot autostart (Docker)
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
User=YOUR_USER
WorkingDirectory=/path/to/2026-robocon-volleyball
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down

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

# 5. 查看状态
sudo systemctl status volleyball-robot.service

# 关闭自启动
sudo systemctl disable volleyball-robot.service
```

