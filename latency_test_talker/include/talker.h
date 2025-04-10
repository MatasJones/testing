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
#include <map>
#include <string>
#include <thread>
#include <time.h>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

#define NB_LISTENERS 1

using namespace std;

class talker : public rclcpp::Node {
public:
  talker();
  double send_time;
  int count_ = 0;

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr exp_timer_;
  rclcpp::TimerBase::SharedPtr switch_off_timer_;

  // Declare the Publishers
  rclcpp::Publisher<custom_msg::msg::CustomString>::SharedPtr talker_publisher_;
  rclcpp::Publisher<custom_msg::msg::SyncMsg>::SharedPtr sync_publisher_;

  // Declare the Subscribers
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      talker_subscriber_;
  rclcpp::Subscription<custom_msg::msg::SyncMsg>::SharedPtr sync_subscriber_;

  double msgs_all_sent_time;
  std::ofstream file;
  std::string logger_name = "/home/testing/dev_ws/src/testing_logs/logger.csv";

  // Experiment parameters
  int repetitions;
  std::vector<int> sizes;
  std::vector<int> ids;
  int current_iteration = 0;
  int current_size = 0;
  int sync_array[NB_LISTENERS] = {0};
  bool terminate_set = false;

  map<tuple<int, int>, tuple<int, int, double>> latency_map;

  std::filesystem::path cwd = std::filesystem::current_path();

  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  void perform_sync();
  void get_response_time(const custom_msg::msg::CustomString::SharedPtr msg);
  void create_logger();
  void setup_experiment();
  void run_experiment();
  void echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg);
  void map_time(int msg_nb, int size, double time);
  void terminate_exp();

  template <typename T> void log(T data);
};

#endif