/*******************************************************************************
 * @file serial_driver.hpp
 * @brief 串口驱动节点类，负责上下位机串口通信
 *******************************************************************************/

#ifndef VOLLEYBALL_SERIAL_DRIVER_HPP
#define VOLLEYBALL_SERIAL_DRIVER_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_srvs/srv/trigger.hpp>
// Project
#include "volleyball_interfaces/msg/robot_base.hpp"
#include "volleyball_interfaces/msg/plan.hpp"
#include "volleyball_interfaces/msg/ball.hpp"
#include "volleyball_interfaces/msg/pid_camera.hpp"
#include "packet.h"

using namespace drivers::serial_driver;
using namespace std::chrono_literals;

/*******************************************************************************
 * @class serial_driver_node
 * @brief 串口驱动节点，负责收发下位机数据并发布ROS消息
 *******************************************************************************/
class serial_driver_node : public rclcpp::Node
{
public:
    serial_driver_node(std::string node_name);  // 构造函数，初始化串口参数与ROS组件
    ~serial_driver_node();                       // 析构函数，关闭串口释放内存

private:
    void serial_reopen_callback();   // 串口断线重连回调（1Hz定时触发）
    void serial_read_thread();       // 串口读取线程，循环接收并校验数据包
    void serial_write(uint8_t *data, size_t len);  // 串口写入原始数据
    void serial_write_callback();    // 定时写入回调，发送规划指令
    uint8_t getContentXor(uint8_t *data_begin, int len);  // 计算异或校验值
    void robot_callback();           // 发布机器人位姿与TF的定时回调
    void plan_callback(volleyball_interfaces::msg::Plan::SharedPtr msg);  // 规划消息订阅回调

    void pid_cam_callback(volleyball_interfaces::msg::PidCamera::SharedPtr msg);
    void ball_callback(volleyball_interfaces::msg::Ball::SharedPtr msg);

    /*@brief 串口相关变量*/
    bool isOpen = false;
    bool isRead = false;
    bool has_new_plan = false;
    bool has_new_pid_cam_ = false;
    bool has_pid_cam_data_ = false;  // IBVS 持续发送模式标志
    bool has_ball_depth_ = false;
    float latest_ball_depth_ = 1000.0f;
    std::string *dev_name;
    std::thread serialReadThread;
    SerialPortConfig *portConfig;
    IoContext ctx;
    SerialDriver serialDriver = SerialDriver(ctx);

    // TF广播器
    double timestamp_offset_ = 0;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 下位机→上位机坐标系转换参数,角度单位deg度
    double odom2base_roll;
    double odom2base_pitch;
    double odom2base_yaw;
    const double DEG2RAD = 3.1415926535 / 180.0;

    // ROS相关变量
    rclcpp::TimerBase::SharedPtr reopenTimer;
    rclcpp::TimerBase::SharedPtr writeTimer;
    rclcpp::TimerBase::SharedPtr publishTimer;
    rclcpp::Publisher<volleyball_interfaces::msg::RobotBase>::SharedPtr robot_base_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<volleyball_interfaces::msg::Plan>::SharedPtr plan_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_planner_client_;
    rclcpp::Subscription<volleyball_interfaces::msg::PidCamera>::SharedPtr pid_cam_sub_;
    rclcpp::Subscription<volleyball_interfaces::msg::Ball>::SharedPtr ball_sub_;
    volleyball_interfaces::msg::Plan latest_plan_;
    volleyball_interfaces::msg::PidCamera latest_pid_cam_;

    uint8_t last_mode_ = 0;  // 上一次下位机模式，用于检测模式切换
    

    // 数据包联合体指针，用于串口字节流与结构体的转换
    robotArray *robotArray_ptr;
    planArray *planArray_ptr;
};

#endif  // VOLLEYBALL_SERIAL_DRIVER_HPP