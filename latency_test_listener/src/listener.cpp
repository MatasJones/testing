#include "listener.h"

#define SOCKET_MODE

#define UDP
// #define TCP

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
#ifdef TCP
  // Create a poll to verify if the socket is ready to read
  struct pollfd fds;
  fds.fd = sockfd;
  fds.events = POLLIN; // Check for incoming data

  while (running) {
    int ret = poll(&fds, 1, 0); // Determine the state of the socket
    // 1) Read incoming data from the socket
    if (ret > 0) {
      if (fds.revents & POLLIN) {
        char buffer[SOCKET_BUFFER_SIZE];
        bzero(buffer, SOCKET_BUFFER_SIZE);
        int n = read(sockfd, buffer, SOCKET_BUFFER_SIZE);
        if (n < 0) {
          // RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
          break;
        }
        // RCLCPP_INFO(this->get_logger(), "%s\n", buffer);
        // Send back data straight away

        // Extract msg number from the message
        std::string str_buffer(buffer);
        if (str_buffer == "SHUTDOWN") {
          RCLCPP_INFO(this->get_logger(), "Shutdown message received");
          running = false;
          break;
        }
        size_t first = str_buffer.find('_');
        size_t second = str_buffer.find('_', first + 1);
        std::string extracted =
            str_buffer.substr(first + 1, second - first - 1);

        // Verify that the msg id is extractable
        if (!(first != std::string::npos && second != std::string::npos &&
              second > first)) {
          // RCLCPP_ERROR(this->get_logger(), "ERROR: invalid message format");
          continue;
        }
        // RCLCPP_INFO(this->get_logger(), "Extracted msg number: %s",
        //             extracted.c_str());
        power =
            ((std::stoi(extracted.c_str()) + 1) % 50) == 0 ? power + 1 : power;
        std::string test_string(std::pow(10, power), 'B');
        // Send the data back to the server
        std::string msg = "C_" + extracted + "_" + test_string;
        strncpy(buffer, msg.c_str(), sizeof(buffer));
        n = write(sockfd, buffer, strlen(buffer));
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
#endif
  // Close the socket
  free(serv_addr);
  serv_addr = NULL;
  close(sockfd);

  RCLCPP_INFO(this->get_logger(), "Socket closed");
  // Shutdown the node
  rclcpp::shutdown();

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

#ifdef TCP
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
#endif

#ifdef UDP
  // Setup UDP socket
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR opening socket");
    return 0;
  }

  // Reset all the serv_addr values to zero
  bzero((char *)&serv_addr, sizeof(serv_addr)); // The function bzero() sets all
                                                // values in a buffer to zero

  // Set the serv_addr parameters
  serv_addr = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
  if (serv_addr == NULL) {
    RCLCPP_ERROR(this->get_logger(), "ERROR allocating memory for serv_addr");
    return 0;
  }
  serv_addr.sin_family = AF_INET; // Means that we are using IPv4
  // Tell the socket to accept any of the host machines IPs
  serv_addr.sin_addr.s_addr = INADDR_ANY; // Tells the kernel to bind the
                                          // socket to all available interfaces
  serv_addr.sin_port =
      htons(port); // This sets the port number the server will listen on,
                   // converting it from host byte order to network byte order

  // UDP client does not need to know its own port, it is handled by the OS as
  // there is no full connection like TCP

  // Send a message to the server
  int sync_msg =
      sendto(sockfd, "First msg", 10, 0, serv_addr, sizeof(struct sockaddr_in));
  if (sync_msg < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR sending message");
    return 0;
  }

  // Wait for a response from the server
  sync_msg = recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)&serv_addr,
                      sizeof(serv_addr)); // n is the number of bytes read
  buffer[sync_msg] = '\0';

  if (sync_msg < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
    return 0;
  }
  if (strncmp(buffer, "SERVER_ACK", 10) != 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: server did not acknowledge");
    return 0;
  }
  RCLCPP_INFO(this->get_logger(), "Message from server: %s", buffer);
  // Send acknowledgment to the server
  sync_msg = sendto(sockfd, "CLIENT_ACK", 10, 0, serv_addr,
                    sizeof(struct sockaddr_in));

  RCLCPP_INFO(this->get_logger(), "UDP socket setup done");

#endif
  return 1;
}
