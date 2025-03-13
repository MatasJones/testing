#include "listener.h"

listener::listener() : Node("listener"), count_(0) {

    RCLCPP_INFO(this->get_logger(), "Creating listener");

    // This block creates a subscriber on a specified topic and binds it to a callback
    this->listener_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        ("/latency_test_talker/COMP_TO_PI"), 10, std::bind(&listener::echo, this, std::placeholders::_1));

    this->listener_publisher_ = this->create_publisher<std_msgs::msg::String>(("PI_TO_COMP"), 10);

    RCLCPP_INFO(this->get_logger(), "Listener created");
}

void listener::echo(const std_msgs::msg::String::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "Msg received from computer! Sending response..., %d", count_++);   
    auto message = std_msgs::msg::String();
    message.data = "Hello from listener! ";
    listener_publisher_->publish(message);

}