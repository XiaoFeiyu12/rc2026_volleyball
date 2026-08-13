# ===============基础镜像===================
FROM ros:humble-ros-base-jammy 
# ===============环境配置===================
ENV DEBIAN_FRONTEND=noninteractive
#================换源======================
RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y curl vim build-essential && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt install curl gnupg2 -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
#================安装依赖===================
#先安装基础源
RUN apt-get update && apt-get install -y \
    # 系统
    wget \
    gpg \
    gnupg \
    lsb-release \
    ca-certificates \
    tzdata \
    #python
    python3 \
    python3-pip \
    python3-yaml \
    #Eigen
    libeigen3-dev \
    #ROS2依赖
    ros-humble-rclcpp \
    ros-humble-cv-bridge \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-foxglove-bridge \
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
    #opencv
    ros-humble-vision-opencv \
    libopencv-dev \
    #删除缓存
    && rm -rf /var/lib/apt/lists/*

#realsenseSDK安装
RUN apt-get update \
    && apt-get install -y \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    && rm -rf /var/lib/apt/lists/*

#openvino APT安装
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
    && apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
    && echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino.list\
    && apt update \
    && apt install -y openvino-2026.2.0 \
    && rm -rf /var/lib/apt/lists/*

#=================设置工作目录===============
WORKDIR /workspace_volleyball_robot

#==================转移代码=================
COPY src ./src
RUN . /opt/ros/humble/setup.sh && colcon build --parallel-workers 4
#==================加载环境变量==============
RUN echo "source /opt/ros/humble/setup.sh" >> /root/.bashrc 

CMD [ "bash" ]

    