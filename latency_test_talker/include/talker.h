#ifndef TALKER_H
#define TALKER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <time.h>

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
      talker_subscriber_;

  std::chrono::time_point<std::chrono::system_clock> start, end;
  std::ofstream file;
  std::string logger_name = "/home/testing/dev_ws/src/testing_logs/logger.csv";
  double sending_time, recieving_time;

  void get_response_time(const custom_msg::msg::CustomString::SharedPtr msg);
  void timer_callback();
  void create_logger();

  template <typename T> void log(T data);
};

#endif