#ifndef TALKER_H
#define TALKER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <time.h>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std;

class talker : public rclcpp::Node {
public:
  talker();
  double send_time;
  int count_ = 0;

private:
  rclcpp::TimerBase::SharedPtr timer_;

  // Declare a Publisher
  rclcpp::Publisher<custom_msg::msg::CustomString>::SharedPtr talker_publisher_;

  // Declare a subscriber
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      sync_subscriber_;

  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      talker_subscriber_;

  // Declare a thread
  std::thread exp_thread_;

  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::ofstream file;
  std::string logger_name = "/home/testing/dev_ws/src/testing_logs/logger.csv";
  double sending_time, recieving_time;

  // Experiment parameters
  int repetitions;
  std::vector<int> sizes;
  int current_iteration = 0;
  int current_size = 0;

  std::filesystem::path cwd = std::filesystem::current_path();

  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  void perform_sync();
  void get_response_time(const custom_msg::msg::CustomString::SharedPtr msg);
  void create_logger();
  void setup_experiment();
  void run_experiment();

  template <typename T> void log(T data);
};

#endif