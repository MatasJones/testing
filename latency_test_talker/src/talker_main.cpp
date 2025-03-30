#include "rclcpp/rclcpp.hpp"
#include "talker.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("latency_test_talker");
  RCLCPP_INFO(node->get_logger(), "Creating node...");

  rclcpp::spin(std::make_shared<talker>());
  rclcpp::shutdown();
  return 0;
}