/*******************************************************************************
 * @file serial_driver.cpp
 * @brief 串口驱动节点实现，负责上下位机通信
 *******************************************************************************/

#include "../include/volleyball_serial_driver/serial_driver.hpp"

/*****************************************************************
 * @brief 构造函数：初始化串口参数、ROS发布订阅、TF广播及读取线程
 *****************************************************************/
SerialDriverNode::SerialDriverNode(std::string node_name)
	: rclcpp::Node(node_name), ctx_{IoContext(2)}, robot_array_ptr_{new RobotArray}, plan_array_ptr_{new PlanArray}
{
	// 获取参数
	dev_name_ = new std::string(this->declare_parameter<std::string>("device_name", "/dev/ttyACM"));
	int baud_rate = this->declare_parameter<int>("baud_rate", 115200);
	timestamp_offset_ = this->declare_parameter<double>("timestamp_offset", 0.006);
	odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
	base_link_frame_id_ = this->declare_parameter<std::string>("base_link_frame_id", "base_link");
	std::vector<double> odom2base_rpy =
		this->declare_parameter<std::vector<double>>("odom2base_rpy", std::vector<double>{});
	odom2base_roll_ = odom2base_rpy[0];
	odom2base_pitch_ = odom2base_rpy[1];
	odom2base_yaw_ = odom2base_rpy[2];
	port_config_ = new SerialPortConfig(baud_rate, FlowControl::NONE, Parity::NONE, StopBits::ONE);

	// 清零数据缓冲区
	memset(robot_array_ptr_->array_, 0, sizeof(RobotArray));
	memset(plan_array_ptr_->array_, 0, sizeof(PlanArray));

	// 串口重启定时器（1Hz）
	reopen_timer_ = this->create_wall_timer(1s, std::bind(&SerialDriverNode::serial_reopen_callback, this));

	// 发布定时器（500Hz）
	publish_timer_ = this->create_wall_timer(10ms, std::bind(&SerialDriverNode::robot_callback, this));

	// TF广播器
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	// 发布机器人位姿消息
	robot_base_pub_ =
		this->create_publisher<volleyball_interfaces::msg::RobotBase>("/serial_driver/robot", rclcpp::SensorDataQoS());
	// 发布joint_states给robot_state_publisher用于delta_arm俯仰角TF
	joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
	// tracker/planner复位服务客户端（下位机切模式时调用）
	reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");
	reset_planner_client_ = this->create_client<std_srvs::srv::Trigger>("/planner/reset");
	// 订阅规划消息
	plan_sub_ = this->create_subscription<volleyball_interfaces::msg::Plan>(
		"/planner/plan", rclcpp::SensorDataQoS(),
		std::bind(&SerialDriverNode::plan_callback, this, std::placeholders::_1));

	// 串口写入定时器（500Hz）
	write_timer_ = this->create_wall_timer(2ms, std::bind(&SerialDriverNode::serial_write_callback, this));

	// 启动串口读取线程
	serial_read_thread_ = std::thread(&SerialDriverNode::serial_read_thread, this);
	serial_read_thread_.detach();

	RCLCPP_INFO(get_logger(), "节点:/%s启动", node_name.c_str());

	// 发布初始TF
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = this->now();
	t.header.frame_id = odom_frame_id_;
	t.child_frame_id = base_link_frame_id_;
	t.transform.translation.x = 0;
	t.transform.translation.y = 0;
	tf2::Quaternion q;
	q.setRPY(odom2base_roll_, odom2base_pitch_, odom2base_yaw_);
	t.transform.rotation = tf2::toMsg(q);
	tf_broadcaster_->sendTransform(t);
	RCLCPP_DEBUG(get_logger(), "x:%05.2f/y:%05.2f/self_yaw:%05.2f", 0.0, 0.0, 0.0);
}

/*****************************************************************
 * @brief 析构函数：关闭串口并释放动态内存
 *****************************************************************/
SerialDriverNode::~SerialDriverNode()
{
	if (serial_driver_.port()->is_open())
	{
		serial_driver_.port()->close();
	}
	// 释放内存
	delete dev_name_;
	delete port_config_;
	delete robot_array_ptr_;
	delete plan_array_ptr_;
}

/*****************************************************************
 * @brief 串口断线重连回调，定时检测串口状态并尝试重新打开
 *****************************************************************/
void SerialDriverNode::serial_reopen_callback()
{
	// 串口失效时尝试重启
	if (!is_open_)
	{
		try
		{
			RCLCPP_WARN(get_logger(), "重启串口:%s...", dev_name_->c_str());
			serial_driver_.init_port(*dev_name_, *port_config_);
			serial_driver_.port()->open();
			is_open_ = serial_driver_.port()->is_open();
		}
		catch (const std::system_error &error)
		{
			RCLCPP_ERROR(get_logger(), "打开串口:%s失败", dev_name_->c_str());
			is_open_ = false;
		}
		if (is_open_) RCLCPP_INFO(get_logger(), "打开串口:%s成功", dev_name_->c_str());
	}
}

/*****************************************************************
 * @brief 串口读取线程，循环接收并校验下位机数据包
 *****************************************************************/
void SerialDriverNode::serial_read_thread()
{
	while (rclcpp::ok())
	{
		std::vector<uint8_t> head(1);
		std::vector<uint8_t> robot_data(sizeof(robot_array_ptr_->array_) - 1);
		if (is_open_)
		{
			try
			{
				serial_driver_.port()->receive(head);
				if (head[0] == 0xAA)
				{  // 包头为0xAA
					serial_driver_.port()->receive(robot_data);
					robot_data.insert(robot_data.begin(), head[0]);

					// 校验环节
					uint8_t tail = robot_data[robot_data.size() - 1];
					uint8_t my_xor = robot_data[robot_data.size() - 2];

					if (tail == 0x55 && my_xor == SerialDriverNode::get_content_xor(&robot_data[1], robot_data.size() - 3))
					{
						memcpy(robot_array_ptr_->array_, robot_data.data(), sizeof(robot_array_ptr_->array_));
						is_read_ = true;
					}
					else
					{
						RCLCPP_WARN(this->get_logger(), "数据包错误");
					}
					//  RCLCPP_INFO(get_logger(), "读取串口.");
				}
			}

			catch (const std::exception &error)
			{
				RCLCPP_ERROR(get_logger(), "读取串口时发生错误.");
				is_open_ = false;
			}
		}
	}
}

/*****************************************************************
 * @brief 串口写入原始数据
 * @param data 待发送数据指针
 * @param len 数据长度
 *****************************************************************/
void SerialDriverNode::serial_write(uint8_t *data, size_t len)
{
	std::vector<uint8_t> temp_data(data, data + len);
	try
	{
		serial_driver_.port()->send(temp_data);
		// RCLCPP_INFO(get_logger(), "写入串口.");
	}
	catch (const std::exception &error)
	{
		RCLCPP_ERROR(get_logger(), "写入串口时发生错误.");
		is_open_ = false;
	}
}

/*****************************************************************
 * @brief 串口写入定时回调（500Hz），检查规划指令并发送
 *****************************************************************/
void SerialDriverNode::serial_write_callback()
{
	if (is_open_ && plan_array_ptr_ != nullptr)
	{
		plan_array_ptr_->msg_.header_ = 0xAA;
		plan_array_ptr_->msg_.cmd_ = 0;
		plan_array_ptr_->msg_.len_ = 16;
		plan_array_ptr_->msg_.x_ = latest_plan_.x;
		plan_array_ptr_->msg_.y_ = latest_plan_.y;
		plan_array_ptr_->msg_.is_hit_ = 0;
		plan_array_ptr_->msg_.landing_time_ = latest_plan_.landing_time - timestamp_offset_;  // 减去传输延迟
		plan_array_ptr_->msg_.my_xor_ = get_content_xor(&plan_array_ptr_->array_[1], sizeof(plan_array_ptr_->array_) - 3);
		plan_array_ptr_->msg_.tail_ = 0x55;
		serial_write(plan_array_ptr_->array_, sizeof(plan_array_ptr_->array_));
		RCLCPP_INFO(this->get_logger(), "下位机发布规划:dx:%.1f,dy:.%1f", latest_plan_.x, latest_plan_.y);
		has_new_plan_ = false;
	}
}

/*****************************************************************
 * @brief 发布机器人位姿及TF的定时回调（500Hz）
 *****************************************************************/
void SerialDriverNode::robot_callback()
{
	if (is_open_ && is_read_)
	{
		try
		{
			// 从下位机获取delta机械臂俯仰角,通过JointState发布TF
			double pitch = robot_array_ptr_->msg_.self_pitch_;

			RCLCPP_DEBUG(this->get_logger(), "x:%05.2f/y:%05.2f/self_yaw:%05.2f/self_pitch:%05.2f,mode:%d",
						 robot_array_ptr_->msg_.x_, robot_array_ptr_->msg_.y_, robot_array_ptr_->msg_.self_yaw_,
						 robot_array_ptr_->msg_.self_pitch_, robot_array_ptr_->msg_.mode_);

			// 发布delta_arm_joint关节状态,robot_state_publisher根据URDF自动发布base_link->delta_arm_link的TF
			// URDF中已定义静态偏移: xyz="0.051 0 0.191" rpy="-3.1416 0 0", joint axis="0 1 0"(绕Y轴俯仰)
			sensor_msgs::msg::JointState joint_state;
			joint_state.header.stamp = this->now();
			joint_state.name = {"delta_arm_joint"};
			joint_state.position = {-pitch};
			joint_state_pub_->publish(joint_state);

			// 发布机器人基座消息（含模式信息）
			volleyball_interfaces::msg::RobotBase robot_msg;
			robot_msg.x = robot_array_ptr_->msg_.x_;
			robot_msg.y = robot_array_ptr_->msg_.y_;
			robot_msg.self_pitch = pitch;
			robot_msg.self_yaw = robot_array_ptr_->msg_.self_yaw_;
			robot_msg.mode = robot_array_ptr_->msg_.mode_;
			robot_base_pub_->publish(robot_msg);

			// 检测下位机模式从自主模式切换至遥控模式，触发tracker和planner复位
			uint8_t current_mode = robot_array_ptr_->msg_.mode_;
			if (last_mode_ == 2 && current_mode != last_mode_)
			{
				RCLCPP_INFO(this->get_logger(), "检测到下位机模式切换: %d -> %d，调用tracker/planner复位服务",
							last_mode_, current_mode);

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

			is_read_ = false;
		}
		catch (const std::exception &ex)
		{
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 20, "处理串口数据时发生错误: %s", ex.what());
		}
	}
}

/*****************************************************************
 * @brief 规划消息订阅回调，将规划指令打包为下位机协议格式
 *****************************************************************/
void SerialDriverNode::plan_callback(volleyball_interfaces::msg::Plan::SharedPtr msg)
{
	latest_plan_ = *msg;
	has_new_plan_ = true;
}


uint8_t SerialDriverNode::get_content_xor(uint8_t *data_begin, int len)
{
	uint8_t my_xor = 0;
	while (len--)
	{
		my_xor ^= *data_begin++;
	}
	return my_xor;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	std::shared_ptr<SerialDriverNode> node = std::make_shared<SerialDriverNode>("serial_driver_node");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
