#include "talker.h"

talker::talker() : Node("talker"), count_(0) {

  RCLCPP_INFO(this->get_logger(), "Creating talker");

  // This block creates a subscriber on a specified topic and binds it to a
  // callback
  this->talker_subscriber_ =
      this->create_subscription<custom_msg::msg::Int16msg>(
          ("/latency_test_listener/PI_TO_COMP"), 10,
          std::bind(&talker::get_response_time, this, std::placeholders::_1));

  // Create a publisher on a specified topic
  this->talker_publisher_ =
      this->create_publisher<custom_msg::msg::Int16msg>(("COMP_TO_PI"), 10);

  // This block creates a publisher on a specified topic and binds it to a
  // callback
  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(3000),
                              std::bind(&talker::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Talker created");
}

void talker::get_response_time(const custom_msg::msg::Int16msg::SharedPtr msg) {

  // this line allows to add a delay in seconds in case of testing
  // std::this_thread::sleep_for(std::chrono::seconds(3));

  // Get the current time in microseconds
  recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;

  // substract the recieve and send timestamps to get elapsed time in
  // microseconds
  double elapsed_time = recieving_time - sending_time;

  // Log the elapsed time in microseconds
  RCLCPP_INFO(this->get_logger(), "Msg received from listener: %d",
              static_cast<int>(msg->num));
  RCLCPP_INFO(this->get_logger(), "Elapsed time: %f ms", elapsed_time);
}

void talker::timer_callback() {

  // Create a message to be sent to the listener and count iterration
  auto message = custom_msg::msg::Int16msg();
  message.num = 54;
  RCLCPP_INFO(this->get_logger(), "Talker count: %d", count_);

  //  Get the current ROS 2 time in milliseconds as a double
  sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;

  // Publish message on topic
  talker_publisher_->publish(message);
}