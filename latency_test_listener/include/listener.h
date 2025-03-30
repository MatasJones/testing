#ifndef LISTENER_H
#define LISTENER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sync_service/srv/sync_check.hpp"

#include <chrono>
#include <iostream>

class listener : public rclcpp::Node {

public:
  listener();
  int count_ = 0;

private:
  // Declare a service server
  rclcpp::Service<sync_service::srv::SyncCheck>::SharedPtr sync_service_;

  // Declare a Publisher
  rclcpp::Publisher<custom_msg::msg::CustomString>::SharedPtr
      listener_publisher_;

  // Declare a subscriber
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      listener_subscriber_;

  void sync_response(
      const std::shared_ptr<sync_service::srv::SyncCheck::Request> request,
      std::shared_ptr<sync_service::srv::SyncCheck::Response> response);
  void echo(const custom_msg::msg::CustomString::SharedPtr msg);
};

#endif