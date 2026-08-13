#include "volleyball_detect/detector_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<volleyball::DetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}