#ifndef TALKER_H
#define TALKER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <iostream>
#include <thread>

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

  double sending_time, recieving_time;

  void get_response_time(const custom_msg::msg::CustomString::SharedPtr msg);
  void timer_callback();
};

#endif