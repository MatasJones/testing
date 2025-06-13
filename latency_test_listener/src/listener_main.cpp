#include "comp.h"
#include "listener.h"
#include "rclcpp/rclcpp.hpp"

#define COMP
// #define TEST

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("latency_test_listener");
  RCLCPP_INFO(node->get_logger(), "Creating node...");

#ifdef TEST
  rclcpp::spin(std::make_shared<listener>());
#endif

#ifdef COMP
  rclcpp::spin(std::make_shared<comp>());
#endif

  rclcpp::shutdown();
  return 0;
}