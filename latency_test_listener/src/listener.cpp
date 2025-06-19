#include "listener.h"

// #define OG_QOS_MODE
#define CUSTOM_QOS_MODE
// #define LOG_MODE

listener::listener() : Node("listener"), count_(0) {

  RCLCPP_INFO(this->get_logger(), "Creating listener");
  get_ip_addr();

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

// This block creates a subscriber on a specified topic and binds it to a
// callback
#ifdef CUSTOM_QOS_MODE
  this->listener_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_talker/COMP_TO_PI"), custom_qos,
          std::bind(&listener::echo, this, std::placeholders::_1));

  this->listener_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("PI_TO_COMP"),
                                                            custom_qos);

#endif

#ifdef OG_QOS_MODE

  this->listener_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_talker/COMP_TO_PI"), 10,
          std::bind(&listener::echo, this, std::placeholders::_1));

  this->listener_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("PI_TO_COMP"), 10);

#endif

  this->sync_subscriber_ = this->create_subscription<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_OUT"), QUEUE_SIZE,
      std::bind(&listener::echo_sync, this, std::placeholders::_1));

  this->sync_publisher_ = this->create_publisher<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_IN"), QUEUE_SIZE);

  RCLCPP_INFO(this->get_logger(), "Listener created");
}

void listener::echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Sync message received");
  if (msg->message == "SHUTDOWN") {
    RCLCPP_INFO(this->get_logger(), "Shutdown message received");
    rclcpp::shutdown();
  }
  auto message = custom_msg::msg::SyncMsg();
  std::string sync_message = msg->message;
  if (sync_message == "SYNC_CHECK") {
    RCLCPP_INFO(this->get_logger(), "Sync check received");
    message.message = "SYNC_CHECK";
    sync_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Sync message sent");
  }
}

void listener::echo(const custom_msg::msg::CustomString::SharedPtr msg) {
  auto message = custom_msg::msg::CustomString();
  std::string test_string(1, 'B');
  // std::string test_string(msg->size, 'B');
  message.message = test_string;
  message.size = msg->size;
  message.msg_id = msg->msg_id;

  listener_publisher_->publish(message);
#ifdef LOG_MODE
  RCLCPP_INFO(this->get_logger(),
              "Msg %d of size %d recieved. Sending response...", msg->msg_id,
              msg->size);
#endif
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

  // Extract the ID from the config file
  id = config["id"][ip_addr].as<int>();
  RCLCPP_INFO(this->get_logger(), "ID for IP %s: %d", ip_addr.c_str(), id);
}
