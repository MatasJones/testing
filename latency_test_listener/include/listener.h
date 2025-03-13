#ifndef LISTENER_H
#define LISTENER_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>    
#include <chrono>

class listener : public rclcpp :: Node {

    public:
        listener();
        int count_ = 0;

    private:

        // Declare a Publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr listener_publisher_;

        // Declare a subscriber
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listener_subscriber_;

        void echo(const std_msgs::msg::String::SharedPtr msg);
};

#endif