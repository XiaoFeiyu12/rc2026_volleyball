/*******************************************************************************
 * @file serial_driver.cpp
 * @brief 串口驱动节点实现，负责上下位机通信
 *******************************************************************************/

#include "../include/volleyball_serial_driver/serial_driver.hpp"

/*****************************************************************
 * @brief 构造函数：初始化串口参数、ROS发布订阅、TF广播及读取线程
 *****************************************************************/
serial_driver_node::serial_driver_node(std::string node_name)
  : rclcpp::Node(node_name), ctx{ IoContext(2) }, robotArray_ptr{ new robotArray }, planArray_ptr{ new planArray }
{
    // 获取参数
    dev_name = new std::string(this->declare_parameter<std::string>("device_name", "/dev/ttyACM"));
    int baud_rate = this->declare_parameter<int>("baud_rate", 115200);
    timestamp_offset_ = this->declare_parameter<double>("timestamp_offset", 0.006);
    odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
    base_link_frame_id_ = this->declare_parameter<std::string>("base_link_frame_id", "base_link");
    std::vector<double> odom2base_rpy = this->declare_parameter<std::vector<double>>("odom2base_rpy", std::vector<double>{});
    odom2base_roll = odom2base_rpy[0];
    odom2base_pitch = odom2base_rpy[1];
    odom2base_yaw = odom2base_rpy[2];
    portConfig = new SerialPortConfig(baud_rate, FlowControl::NONE, Parity::NONE, StopBits::ONE);

    // 清零数据缓冲区
    memset(robotArray_ptr->array, 0, sizeof(robotArray));
    memset(planArray_ptr->array, 0, sizeof(planArray));

    // 串口重启定时器（1Hz）
    reopenTimer = this->create_wall_timer(1s, std::bind(&serial_driver_node::serial_reopen_callback, this));

    // 发布定时器（500Hz）
    publishTimer = this->create_wall_timer(10ms, std::bind(&serial_driver_node::robot_callback, this));

    // TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 发布机器人位姿消息
    robot_base_pub_ =
        this->create_publisher<volleyball_interfaces::msg::RobotBase>("/serial_driver/robot", rclcpp::SensorDataQoS());
    // 发布joint_states给robot_state_publisher用于delta_arm俯仰角TF
    joint_state_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    // tracker/planner复位服务客户端（下位机切模式时调用）
    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");
    reset_planner_client_ = this->create_client<std_srvs::srv::Trigger>("/planner/reset");
    // 订阅规划消息
    plan_sub_ = this->create_subscription<volleyball_interfaces::msg::Plan>(
        "/planner/plan", rclcpp::SensorDataQoS(), std::bind(&serial_driver_node::plan_callback, this, std::placeholders::_1));
    // 订阅 /detector/ball 获取 Realsense 深度
    ball_sub_ = this->create_subscription<volleyball_interfaces::msg::Ball>(
        "/detector/ball", rclcpp::SensorDataQoS(), std::bind(&serial_driver_node::ball_callback, this, std::placeholders::_1));

    // 串口写入定时器（500Hz）
    writeTimer = this->create_wall_timer(2ms, std::bind(&serial_driver_node::serial_write_callback, this));

    // 启动串口读取线程
    serialReadThread = std::thread(&serial_driver_node::serial_read_thread, this);
    serialReadThread.detach();

    RCLCPP_INFO(get_logger(), "节点:/%s启动", node_name.c_str());

    // 发布初始TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = odom_frame_id_;
    t.child_frame_id = base_link_frame_id_;
    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    tf2::Quaternion q;
    q.setRPY(odom2base_roll, odom2base_pitch, odom2base_yaw);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
    RCLCPP_DEBUG(get_logger(), "x:%05.2f/y:%05.2f/self_yaw:%05.2f", 0.0, 0.0, 0.0);
}

/*****************************************************************
 * @brief 析构函数：关闭串口并释放动态内存
 *****************************************************************/
serial_driver_node::~serial_driver_node()
{
    if (serialDriver.port()->is_open())
    {
        serialDriver.port()->close();
    }
    // 释放内存
    delete dev_name;
    delete portConfig;
    delete robotArray_ptr;
    delete planArray_ptr;
}

/*****************************************************************
 * @brief 串口断线重连回调，定时检测串口状态并尝试重新打开
 *****************************************************************/
void serial_driver_node::serial_reopen_callback()
{
    // 串口失效时尝试重启
    if (!isOpen)
    {
        try
        {
        RCLCPP_WARN(get_logger(), "重启串口:%s...", dev_name->c_str());
        serialDriver.init_port(*dev_name, *portConfig);
        serialDriver.port()->open();
        isOpen = serialDriver.port()->is_open();
        }
        catch (const std::system_error& error)
        {
        RCLCPP_ERROR(get_logger(), "打开串口:%s失败", dev_name->c_str());
        isOpen = false;
        }
        if (isOpen)
        RCLCPP_INFO(get_logger(), "打开串口:%s成功", dev_name->c_str());
    }
}

/*****************************************************************
 * @brief 串口读取线程，循环接收并校验下位机数据包
 *****************************************************************/
void serial_driver_node::serial_read_thread()
{
    while (rclcpp::ok())
    {
        std::vector<uint8_t> head(1);
        std::vector<uint8_t> robotData(sizeof(robotArray_ptr->array) - 1);
        if (isOpen)
        {
        try
        {
            serialDriver.port()->receive(head);
            if (head[0] == 0xAA)
            {  // 包头为0xAA
                serialDriver.port()->receive(robotData);
                robotData.insert(robotData.begin(), head[0]);

                //校验环节
                uint8_t tail = robotData[robotData.size() - 1];
                uint8_t myxor = robotData[robotData.size() - 2];

                if (tail == 0x55 && myxor == serial_driver_node::getContentXor(&robotData[1], robotData.size() - 3))
                {
                    memcpy(robotArray_ptr->array, robotData.data(), sizeof(robotArray_ptr->array));
                    isRead = true;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "数据包错误");
                }
            //  RCLCPP_INFO(get_logger(), "读取串口.");
            }
            // //调试上位向下位固定发送指令
            // planArray_ptr->msg.header = 0xAA;
            // planArray_ptr->msg.cmd = 0;
            // planArray_ptr->msg.len = 16;
            // planArray_ptr->msg.x = 0.01;
            // planArray_ptr->msg.y = 0.01;
            // planArray_ptr->msg.self_yaw = 0;
            // planArray_ptr->msg.landing_time = 1;  // 更新计划消息中的时间戳,以便计算传输延迟
            // planArray_ptr->msg.my_xor = getContentXor(&planArray_ptr->array[1], sizeof(planArray_ptr->array) - 3);  // 更新校验值
            // planArray_ptr->msg.tail = 0x55;
            // serial_write(planArray_ptr->array, sizeof(planArray_ptr->array));
        }


        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "读取串口时发生错误.");
            isOpen = false;
        }
        }
    }
}

/*****************************************************************
 * @brief 串口写入原始数据
 * @param data 待发送数据指针
 * @param len 数据长度
 *****************************************************************/
void serial_driver_node::serial_write(uint8_t* data, size_t len)
{
  std::vector<uint8_t> tempData(data, data + len);
  try
  {
    serialDriver.port()->send(tempData);
    // RCLCPP_INFO(get_logger(), "写入串口.");
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_logger(), "写入串口时发生错误.");
    isOpen = false;
  }
}

/*****************************************************************
 * @brief 串口写入定时回调（500Hz），检查规划指令并发送
 *****************************************************************/
void serial_driver_node::serial_write_callback()
{
  if (isOpen && planArray_ptr != nullptr)
  {
        planArray_ptr->msg.header = 0xAA;
        planArray_ptr->msg.cmd = 0;
        planArray_ptr->msg.len = 16;
        planArray_ptr->msg.x = latest_plan_.x;
        planArray_ptr->msg.y = latest_plan_.y;
        planArray_ptr->msg.self_yaw = latest_plan_.self_yaw;
        planArray_ptr->msg.landing_time = latest_plan_.landing_time - timestamp_offset_;  // 减去传输延迟
        planArray_ptr->msg.my_xor = getContentXor(&planArray_ptr->array[1], sizeof(planArray_ptr->array) - 3);
        planArray_ptr->msg.tail = 0x55;
        serial_write(planArray_ptr->array, sizeof(planArray_ptr->array));
        RCLCPP_INFO(this->get_logger(),"下位机发布规划:dx:%.1f,dy:.%1f",latest_plan_.x,latest_plan_.y);
        has_new_plan = false;
  }
}

/*****************************************************************
 * @brief 发布机器人位姿及TF的定时回调（500Hz）
 *****************************************************************/
void serial_driver_node::robot_callback()
{
    if (isOpen && isRead)
    {
        try
        {
            // 从下位机获取delta机械臂俯仰角,通过JointState发布TF
            //double pitch = robotArray_ptr->msg.self_pitch;

            RCLCPP_DEBUG(this->get_logger(), "x:%05.2f/y:%05.2f/self_pitch:%05.2f,mode:%d", robotArray_ptr->msg.x, robotArray_ptr->msg.y, robotArray_ptr->msg.self_yaw, robotArray_ptr->msg.mode);

            // // 发布delta_arm_joint关节状态,robot_state_publisher根据URDF自动发布base_link->delta_arm_link的TF
            // // URDF中已定义静态偏移: xyz="0.051 0 0.191" rpy="-3.1416 0 0", joint axis="0 1 0"(绕Y轴俯仰)
            // sensor_msgs::msg::JointState joint_state;
            // joint_state.header.stamp = this->now();
            // joint_state.name = {"delta_arm_joint"};
            // joint_state.position = {-pitch};
            // joint_state_pub_->publish(joint_state);

            // 发布机器人基座消息（含模式信息）
            volleyball_interfaces::msg::RobotBase robot_msg;
            robot_msg.x = robotArray_ptr->msg.x;
            robot_msg.y = robotArray_ptr->msg.y;
            robot_msg.self_pitch = 0.0;
            robot_msg.self_yaw = 0.0;
            robot_msg.mode = robotArray_ptr->msg.mode;
            robot_base_pub_->publish(robot_msg);

            // 检测下位机模式从自主模式切换至遥控模式，触发tracker和planner复位
            uint8_t current_mode = robotArray_ptr->msg.mode;
            if (last_mode_ == 2 && current_mode != last_mode_)
            {
                RCLCPP_INFO(this->get_logger(),
                    "检测到下位机模式切换: %d -> %d，调用tracker/planner复位服务", last_mode_, current_mode);

                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

                if (reset_tracker_client_->wait_for_service(1s))
                {
                    reset_tracker_client_->async_send_request(request);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "tracker复位服务不可用");
                }

                if (reset_planner_client_->wait_for_service(1s))
                {
                    reset_planner_client_->async_send_request(request);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "planner复位服务不可用");
                }
            }
            last_mode_ = current_mode;

            isRead = false;
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 20, "处理串口数据时发生错误: %s", ex.what());
        }
    }
}

/*****************************************************************
 * @brief 规划消息订阅回调，将规划指令打包为下位机协议格式
 *****************************************************************/
void serial_driver_node::plan_callback(volleyball_interfaces::msg::Plan::SharedPtr msg)
{

    latest_plan_ = *msg;
    has_new_plan = true;
}

void serial_driver_node::ball_callback(volleyball_interfaces::msg::Ball::SharedPtr msg)
{
    // 使用 Ball.z（相机坐标系前方深度）作为深度判断依据
    latest_ball_depth_ = msg->z;
    has_ball_depth_ = true;
}

uint8_t serial_driver_node::getContentXor(uint8_t* data_begin, int len)
{
    uint8_t my_xor = 0;
    while (len--)
    {
        my_xor ^= *data_begin++;
    }
    return my_xor;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<serial_driver_node> node = std::make_shared<serial_driver_node>("serial_driver_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
