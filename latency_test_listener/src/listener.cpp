#include "listener.h"

#define SOCKET_MODE

listener::listener() : Node("listener"), count_(0) {

  RCLCPP_INFO(this->get_logger(), "Creating listener");
  get_ip_addr();

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  // This block creates a subscriber on a specified topic and binds it to a
  // callback
  this->listener_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_talker/COMP_TO_PI"), custom_qos,
          std::bind(&listener::echo, this, std::placeholders::_1));

  this->sync_subscriber_ = this->create_subscription<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_OUT"), QUEUE_SIZE,
      std::bind(&listener::echo_sync, this, std::placeholders::_1));

  this->listener_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("PI_TO_COMP"),
                                                            custom_qos);

  this->sync_publisher_ = this->create_publisher<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_IN"), QUEUE_SIZE);

#ifdef SOCKET_MODE
  RCLCPP_INFO(this->get_logger(), "Setting up socket...");
  // Create a socket and connect to the server
  if (!listener::socket_setup()) {
    RCLCPP_INFO(this->get_logger(), "Socket setup failed");
    rclcpp::shutdown();

  } else {
    RCLCPP_INFO(this->get_logger(), "Socket setup done");
  }
  RCLCPP_INFO(this->get_logger(), "Let's rumble!");

  // Create a poll to verify if the socket is ready to read
  struct pollfd fds;
  fds.fd = sockfd;
  fds.events = POLLIN; // Check for incoming data

  while (running) {
    int ret = poll(&fds, 1, 0); // Determine the state of the socket
    // 1) Read incoming data from the socket
    if (ret > 0) {
      if (fds.revents & POLLIN) {
        char buffer[256];
        bzero(buffer, 256);
        int n = read(sockfd, buffer, 255);
        if (n < 0) {
          RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
          break;
        }
        RCLCPP_INFO(this->get_logger(), "%s\n", buffer);
        // Send back data straight away

        // Extract msg number from the message
        std::string str_buffer(buffer);
        size_t first = str_buffer.find('_');
        size_t second = str_buffer.find('_', first + 1);
        std::string extracted =
            str_buffer.substr(first + 1, second - first - 1);

        // Send the data back to the server
        std::string msg = "C_" + extracted + "_";
        strncpy(buffer, msg.c_str(), sizeof(buffer));
        n = write(sockfd, buffer, strlen(buffer));
      }
    }
  }
  // Close the socket
  close(sockfd);

#endif
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
  std::string test_string(msg->size, 'B');
  message.message = test_string;
  message.size = msg->size;
  message.msg_nb = msg->msg_nb;

  listener_publisher_->publish(message);
  // RCLCPP_INFO(this->get_logger(),
  //             "Msg %d of size %d recieved. Sending response...", msg->msg_nb,
  //             msg->size);
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

bool listener::socket_setup() {

  // Variable setup
  int n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[256];

  // Create a socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR opening socket");
  }

  // Get the server address and search for it
  server = gethostbyname(server_ip);
  if (server == NULL) {
    RCLCPP_ERROR(this->get_logger(), "ERROR, no such host");
    return 0;
  }
  /// Initialize the server address
  // Set the socket parameters
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(port);

  /// Attempt connection to server
  if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR connecting");
    return 0;
  }

  // Generate the data to be sent
  bzero(buffer, 256);
  strncpy(buffer, "CLIENT_ACK", sizeof(buffer));
  n = write(sockfd, buffer, strlen(buffer));
  if (n < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR writing to socket");
    return 0;
  }

  // Read the response from the server
  bzero(buffer, 256);
  n = read(sockfd, buffer, 255);
  if (n < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
    return 0;
  }
  RCLCPP_INFO(this->get_logger(), "%s\n", buffer);
  if (strncmp(buffer, "SERVER_ACK", sizeof(buffer)) != 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: server did not acknowledge");
    return 0;
  }
  return 1;
}
