#ifndef TALKER_H
#define TALKER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "custom_msg/msg/sync_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sync_service/srv/sync_check.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <time.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#define NB_LISTENERS 2

using namespace std;

class talker : public rclcpp::Node {
public:
  talker();
  double send_time;
  int count_ = 0;

private:
  rclcpp::TimerBase::SharedPtr timer_;

  // Declare the Publishers
  rclcpp::Publisher<custom_msg::msg::CustomString>::SharedPtr talker_publisher_;
  rclcpp::Publisher<custom_msg::msg::SyncMsg>::SharedPtr sync_publisher_;

  // Declare a service client
  rclcpp::Client<sync_service::srv::SyncCheck>::SharedPtr sync_client_;

  // Declare the Subscribers
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      talker_subscriber_;
  rclcpp::Subscription<custom_msg::msg::SyncMsg>::SharedPtr sync_subscriber_;

  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::ofstream file;
  std::string logger_name = "/home/testing/dev_ws/src/testing_logs/logger.csv";
  double sending_time, recieving_time;

  // Experiment parameters
  int repetitions;
  std::vector<int> sizes;
  std::vector<int> ids;
  int current_iteration = 0;
  int current_size = 0;
  int sync_array[NB_LISTENERS] = {0};

  std::filesystem::path cwd = std::filesystem::current_path();

  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  void perform_sync();
  void get_response_time(const custom_msg::msg::CustomString::SharedPtr msg);
  void create_logger();
  void setup_experiment();
  void run_experiment();
  void echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg);

  template <typename T> void log(T data);
};

#endif