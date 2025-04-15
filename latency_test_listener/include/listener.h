#ifndef LISTENER_H
#define LISTENER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "custom_msg/msg/sync_msg.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sync_service/srv/sync_check.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <netdb.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h> // gethostname
#include <yaml-cpp/yaml.h>

#define QUEUE_SIZE 100

class listener : public rclcpp::Node {

public:
  listener();
  int count_ = 0;

private:
  // Declare a Publisher
  rclcpp::Publisher<custom_msg::msg::CustomString>::SharedPtr
      listener_publisher_;

  rclcpp::Publisher<custom_msg::msg::SyncMsg>::SharedPtr sync_publisher_;

  // Declare a subscriber
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      listener_subscriber_;

  rclcpp::Subscription<custom_msg::msg::SyncMsg>::SharedPtr sync_subscriber_;

  void echo(const custom_msg::msg::CustomString::SharedPtr msg);
  void echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg);
  void get_ip_addr();
  bool socket_setup();

  std::string ip_addr;
  std::filesystem::path cwd = std::filesystem::current_path();
  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  int id;
  int port = 5000;
  char server_ip[13] = "192.168.0.72";
  int sockfd;

  bool running = true;
};

#endif