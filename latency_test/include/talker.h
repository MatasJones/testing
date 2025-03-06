#ifndef TALKER_H
#define TALKER_H


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>    
#include <chrono> 

class talker : public rclcpp::Node{
    public:

        talker();
        double send_time;
        int count_ = 0;
        
    private:

        rclcpp::TimerBase::SharedPtr timer_;

        // Declare a Publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr talker_publisher_;

        // Declare a subscriber
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr talker_subscriber_;

        std::chrono::time_point<std::chrono::system_clock> start, end;
        
        void get_response_time(const std_msgs::msg::String::SharedPtr msg);
        void timer_callback();
};

#endif