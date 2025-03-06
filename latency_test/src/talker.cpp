#include "talker.h"

talker::talker(rclcpp::Node* parent) : parant(parent) {

    RCLCPP_INFO(parent->get_logger(), "Creating talker");

    // This block creates a subscriber on a specified topic and binds it to a callback
    this->talker_subscriber_ = parent->create_subscription<std_msgs::msg::String>
        ("PI_TO_COMP"), 10, std::bind(&talker::get_response_time, this, this->send_time);

    // This block creates a publisher on a specified topic and binds it to a callback
    this->talker_publiher_ = parent->create_subscription<std_msgs::msg::String>("COMP_TO_PI", 10);
    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&talker::timer_callback, this));

    RCLCPP_INFO(parent->get_logger(), "Talker created");
}

talker::get_response_time() {

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = this->end - this->start;
    RCLCPP_INFO(parent->get_logger(), "Elapsed time: %d", elapsed_seconds);
}

talker::timer_callback() {

    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    this->start = std::chrono::system_clock::now();
    publisher_->publish(message);
}