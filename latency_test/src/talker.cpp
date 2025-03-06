#include "talker.h"

talker::talker() : Node("talker"), count_(0) {

    RCLCPP_INFO(this->get_logger(), "Creating talker");

    // This block creates a subscriber on a specified topic and binds it to a callback
    this->talker_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        ("PI_TO_COMP"), 10, std::bind(&talker::get_response_time, this, std::placeholders::_1));

    this->talker_publisher_ = this->create_publisher<std_msgs::msg::String>(("COMP_TO_PI"), 10);
    // This block creates a publisher on a specified topic and binds it to a callback
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&talker::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Talker created");
}

void talker::get_response_time(const std_msgs::msg::String::SharedPtr msg) {

    end = std::chrono::system_clock::now();
    auto elapsed_duration = this->end - this->start;
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed_duration).count();
    RCLCPP_INFO(this->get_logger(), "Elapsed time: %ld", elapsed_seconds);
}

void talker::timer_callback() {

    auto message = std_msgs::msg::String();
    message.data = "Hello from talker! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    this->start = std::chrono::system_clock::now();
    talker_publisher_->publish(message);
}