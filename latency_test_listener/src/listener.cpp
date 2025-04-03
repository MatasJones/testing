#include "listener.h"

listener::listener() : Node("listener"), count_(0) {

  RCLCPP_INFO(this->get_logger(), "Creating listener");
  get_ip_addr();

  // This block creates a subscriber on a specified topic and binds it to a
  // callback
  this->listener_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_talker/COMP_TO_PI"), 10,
          std::bind(&listener::echo, this, std::placeholders::_1));

  this->sync_subscriber_ = this->create_subscription<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_OUT"), 10,
      std::bind(&listener::echo_sync, this, std::placeholders::_1));

  this->listener_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("PI_TO_COMP"), 10);

  this->sync_publisher_ = this->create_publisher<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_IN"), 10);

  this->sync_service_ = this->create_service<sync_service::srv::SyncCheck>(
      "/latency_test_listener/sync_service",
      std::bind(&listener::sync_response, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Listener created");
}

void listener::echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Sync message received");
  auto message = custom_msg::msg::SyncMsg();
  message.id = msg->id;
  message.message = "Sync check response";

  sync_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Sync message sent");
}

void listener::sync_response(
    const std::shared_ptr<sync_service::srv::SyncCheck::Request> request,
    std::shared_ptr<sync_service::srv::SyncCheck::Response> response) {
  if (!request) {
    // Check if request is null
    RCLCPP_ERROR(this->get_logger(), "Received a null request");
    return;
  }

  bool state = request->request;
  if (state == false) {
    RCLCPP_INFO(this->get_logger(), "Sync request received");
    rclcpp::shutdown();
  }
  response->response = true;
  RCLCPP_INFO(this->get_logger(), "Sync request received");
}

void listener::echo(const custom_msg::msg::CustomString::SharedPtr msg) {

  auto message = custom_msg::msg::CustomString();
  message.id = msg->id;
  std::string test_string(msg->size, 'B');
  message.message = test_string;
  message.size = msg->size;

  listener_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(),
              "Msg received from computer! Sending response..., %d", count_++);
}

void listener::get_ip_addr() {
  char hostname[1024];
  hostname[1023] = '\0';

  // Get the hostname
  if (gethostname(hostname, 1023) == -1) {
    RCLCPP_ERROR(this->get_logger(), "gethostname failed");
  }
  struct addrinfo hints{}, *res, *p;
  hints.ai_family = AF_INET; // IPv4
  hints.ai_socktype = SOCK_STREAM;

  // Get IP address
  if (getaddrinfo(hostname, nullptr, &hints, &res) != 0) {
    RCLCPP_ERROR(this->get_logger(), "getaddrinfo failed");
  }

  // Loop through results and get the first valid IP
  if (res != nullptr) {
    struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
    char ipStr[INET_ADDRSTRLEN];

    inet_ntop(AF_INET, &addr->sin_addr, ipStr, sizeof(ipStr));
    ip_addr = ipStr;
    RCLCPP_INFO(this->get_logger(), "IP address: %s", ip_addr.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "No valid IP address found");
  }
  freeaddrinfo(res); // Free the linked list
  RCLCPP_INFO(this->get_logger(), "config path: %s", config_file_path.c_str());
  YAML::Node config = YAML::LoadFile(config_file_path); // Load the config file
  RCLCPP_INFO(this->get_logger(), "YAML content: %s",
              YAML::Dump(config).c_str());

  // Extract the ID from the config file
  if (config["id"]) {
    if (config["id"]["192.168.0.131"]) {
      int id = config["id"]["192.168.0.131"].as<int>();
      RCLCPP_INFO(this->get_logger(), "ID for IP %s: %d", ip_addr.c_str(), id);
    } else {
      RCLCPP_ERROR(this->get_logger(), "IP address %s not found in config file",
                   ip_addr.c_str());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "'id' section not found in config file");
  }
}
