# VolleyballRobot
Robocon2026 排球机器人上位机算法方案1：落点预测

## 1.项目简介与效果

项目基于 ROS 2 Humble 的排球机器人系统，实现排球检测、追踪、轨迹预测与接球规划的全流程功能。目前由于缺乏调试，效果功能并未真正实现，不稳定。  



---

## 2.架构

![系统架构图](docs/sys_structure.svg)
<center>图1 系统架构图</center>  

---

![数据流图](docs/dataflow.svg)
<center>图2 数据流图</center>

---

![tf树](docs/tf.svg)
<center>图3 tf树</center>


## 3.ROS2包说明
| 包 | 输入 | 输出 | 功能 |
|------|------|------|------|
| **volleyball_detect** | RealSense RGB-D | `/detector/ball` | YOLOv11n + OpenVINO 检测排球 3D 位置 |
| **volleyball_interfaces** | --- | --- | ROS2自定义消息接口 |
| **volleyball_plan** | `/predictor/ball_trajectory` | `/planner/plan` | 计算接球位姿 |
| **volleyball_predict** | `/tracker/target` | `/predictor/ball_trajectory` | 预测球轨迹 |
| **volleyball_robot** | --- | --- | 总启动包，参数文件存放于此 |
| **volleyball_robot_description** | --- | --- | 排球机器人机械描述 |
| **volleyball_serial_driver** | `/planner/plan` | UART 下位机 | 串口协议通信，发布里程计 TF |
| **volleyball_track** | `/detector/ball` | `/tracker/target` | 线性阻力模型 EKF，输出平滑位置和速度 |

>具体原理与参数说明于各功能包内展开说明。
>

## 4.项目目录

drop_point_prediction  
├── docs    // 存放文档、图片
│   └── ...
├── src //ROS2源代码
│   ├── volleyball_detect   // 目标检测模块
│   ├── volleyball_interfaces   //自定义接口
│   ├── volleyball_plan     // 规划模块
│   ├── volleyball_predict  // 预测模块
│   ├── volleyball_robot    // 总启动包，包含参数文件
│   ├── volleyball_robot_description    // 机器人描述
│   ├── volleyball_serial_driver    //串口通信包
│   └── volleyball_track    // 追踪、状态估计
└── tools   //这里基本都是用AI生成的各种脚本，不一定好用
    ├── ball_visualizer.py      //显示track包与观测数据对比图
    ├── extract_img.py          //将ROSbag录包抽帧做数据集
    ├── record_dataset.py       //直接录制并生成数据集
    ├── reload.sh               //重新编译并启动一键命令
    └── test_ball_publisher.py  //生成随机数据测试track、predict、plan包功能

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

