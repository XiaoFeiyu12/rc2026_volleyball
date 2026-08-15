// STL
#include <memory>
// ROS2
#include "rclcpp/rclcpp.hpp"
// Project
#include "volleyball_predict/predictor_node.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<volleyball::PredictNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}