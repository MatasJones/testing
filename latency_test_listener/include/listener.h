#ifndef LISTENER_H
#define LISTENER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "custom_msg/msg/sync_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sync_service/srv/sync_check.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <netdb.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h> // gethostname
#include <yaml-cpp/yaml.h>

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

  rclcpp::Publisher<custom_msg::msg::SyncMsg>::SharedPtr sync_publisher_;

  // Declare a subscriber
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      listener_subscriber_;

  rclcpp::Subscription<custom_msg::msg::SyncMsg>::SharedPtr sync_subscriber_;

  void sync_response(
      const std::shared_ptr<sync_service::srv::SyncCheck::Request> request,
      std::shared_ptr<sync_service::srv::SyncCheck::Response> response);
  void echo(const custom_msg::msg::CustomString::SharedPtr msg);
  void echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg);
  void get_ip_addr();

  std::string ip_addr;
  std::filesystem::path cwd = std::filesystem::current_path();
  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  int id;
};

#endif