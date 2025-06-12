#include "holo.h"
#include "rclcpp/rclcpp.hpp"
#include "talker.h"

// #define HOLO
#define TEST

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("latency_test_talker");
  RCLCPP_INFO(node->get_logger(), "Creating node...");

#ifdef TEST
  rclcpp::spin(std::make_shared<talker>());
#endif

#ifdef HOLO
  rclcpp::spin(std::make_shared<holo>());
#endif

  rclcpp::shutdown();
  return 0;
}