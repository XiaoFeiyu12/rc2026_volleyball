/*******************************************************************************
 * @file ibvs_main.cpp
 * @brief IBVS 控制器节点入口
 *******************************************************************************/

#include "volleyball_ibvs/ibvs_controller_node.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<volleyball::IbvsControllerNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
