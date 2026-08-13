#include <rclcpp/rclcpp.hpp>
#include "volleyball_pid_camera/pid_camera_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<volleyball::PidCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
