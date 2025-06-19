#ifndef TALKER_H
#define TALKER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "custom_msg/msg/sync_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sync_service/srv/sync_check.hpp"

#include "rclcpp/qos.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <time.h>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

#define NB_LISTENERS 1
#define QUEUE_SIZE 100
#define NB_SYNC_CHECKS 10
#define DEAFULT_SPACING 100
#define SYNC_CHECK_PERIOD 200

#define NB_MSGS 50
#define NB_OF_SIZES 5

using namespace std;

class talker : public rclcpp::Node {
public:
  talker();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr exp_timer_;
  rclcpp::TimerBase::SharedPtr test_timer_;

  // Declare the Publishers
  rclcpp::Publisher<custom_msg::msg::CustomString>::SharedPtr talker_publisher_;
  rclcpp::Publisher<custom_msg::msg::SyncMsg>::SharedPtr sync_publisher_;

  // Declare the Subscribers
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      talker_subscriber_;
  rclcpp::Subscription<custom_msg::msg::SyncMsg>::SharedPtr sync_subscriber_;

  int spacing_ms_;

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
  bool sync_check = false;
  int check_count = 0;
  bool test_continue = true;
  uint16_t msg_id = 0;
  int mistach_counter = 0;
  int lost_packes_counter = 0;
  int sync_counter = 0;

  std::tuple<double, double, uint16_t, uint16_t>
      send_receive_time[NB_OF_SIZES * NB_MSGS] = {};

  std::filesystem::path cwd = std::filesystem::current_path();

  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  void perform_sync();
  void get_response_time(const custom_msg::msg::CustomString::SharedPtr msg);
  void create_logger();
  void setup_experiment();
  void run_experiment();
  void echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg);
  void terminate_exp();
  void process_data();
  void test_timer();

  template <typename T> void log(T data);
};

#endif