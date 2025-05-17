#include "listener.h"

// #define UDP
#define TCP

// #define MANUAL_SER
#define FLATBUFF_SER

listener::listener() : Node("listener"), count_(0) {

  RCLCPP_INFO(this->get_logger(), "Creating listener");

  RCLCPP_INFO(this->get_logger(), "Setting up socket...");
  // Create a socket and connect to the server
  if (!listener::socket_setup()) {
    RCLCPP_INFO(this->get_logger(), "Socket setup failed");
    rclcpp::shutdown();

  } else {
    RCLCPP_INFO(this->get_logger(), "Socket setup done");
  }

#ifdef FLATBUFF_SER
  RCLCPP_INFO(this->get_logger(), "Setting up flatbuffers...");
  // Create the flatbuffer builder
  flatbuffers::FlatBufferBuilder builder(1024);

  char test_buffer[SOCKET_BUFFER_SIZE];
  uint32_t size;
  // Create the flatbuffer message
  custom_ser::ser_msg("TEST", 1, 42, &builder, (uint8_t *)&test_buffer, &size);
  // Deserialize the flatbuffer message
  std::string msg;
  uint8_t id;
  int32_t value;
  if (!custom_ser::deser_msg((uint8_t *)&test_buffer, msg, id, value)) {
    RCLCPP_ERROR(this->get_logger(), "Error deserializing flatbuffer message");
  } else {
    if (msg == "TEST" && id == 1 && value == 42) {
      RCLCPP_INFO(this->get_logger(), "Flatbuffer working");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "ERROR DURING FLATBUFFER INITIALIZATION");
    }
  }
#endif

  RCLCPP_INFO(this->get_logger(), "Let's rumble!");

#ifdef TCP
  // Create a poll to verify if the socket is ready to read
  struct pollfd fds;
  fds.fd = sockfd;
  fds.events = POLLIN; // Check for incoming data

  char buffer[SOCKET_BUFFER_SIZE];
  char custom_buffer[1000000];

  while (running) {
    int ret = poll(&fds, 1, 0); // Determine the state of the socket
    // 1) Read incoming data from the socket
    if (ret > 0) {
      if (fds.revents & POLLIN) {

#ifdef MANUAL_SER
        bzero(buffer, SOCKET_BUFFER_SIZE);
        // Read the data from the socket
        int n = read(sockfd, buffer, SOCKET_BUFFER_SIZE);
        if (n < 0) {
          break;
        }
        // Extract msg number from the message
        // If msg = "SHUTDOWN", shutdown the node
        std::string str_buffer(buffer);
        if (str_buffer == "SHUTDOWN") {
          RCLCPP_INFO(this->get_logger(), "Shutdown message received");
          running = false;
          break;
        }
        // Else extract the msg id
        size_t first = str_buffer.find('_');
        size_t second = str_buffer.find('_', first + 1);
        std::string extracted =
            str_buffer.substr(first + 1, second - first - 1);

        // Verify that the msg id is extractable
        if (!(first != std::string::npos && second != std::string::npos &&
              second > first)) {
          continue;
        }

        // Create a new msg of same id and length
        power =
            ((std::stoi(extracted.c_str()) + 1) % 50) == 0 ? power + 1 : power;
        std::string test_string(std::pow(10, power), 'B');
        // Send the data back to the server
        std::string msg = "C_" + extracted + "_" + test_string;
        strncpy(buffer, msg.c_str(), sizeof(buffer));
        // Wirte msg to the server
        n = write(sockfd, buffer, strlen(buffer));
#endif

#ifdef FLATBUFF_SER

        // Read the data from the socket
        int n = read(sockfd, buffer, SOCKET_BUFFER_SIZE);
        if (n < 0) {
          break;
        }

        // for (int i = 0; i < 48; i++) {
        //   RCLCPP_INFO(this->get_logger(), "%d", buffer[i]);
        // }
        RCLCPP_INFO(this->get_logger(), "received message: %s", buffer);
        continue;


        // Send response to server
        if (!custom_ser::deser_msg((uint8_t *)custom_buffer, msg, id, value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Error deserializing flatbuffer message!");
          if (failure_counter++ > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            running = false;
          }
          continue;
        }
        failure_counter = 0;

        RCLCPP_INFO(this->get_logger(), "Received buffer: ");

        if (msg == "SHUTDOWN") {
          RCLCPP_INFO(this->get_logger(), "Shutdown message received");
          running = false;
          std::this_thread::sleep_for(std::chrono::seconds(5));
          continue;

        } else if (msg == "GRACE") {
          RCLCPP_INFO(this->get_logger(), "Grace message received");
          msg = "GRACE_ACK";
        }

        else {
          std::string test_string(std::pow(10, value), 'B');
          msg = test_string;
        }

        custom_ser::ser_msg(msg, id, value, &builder, (uint8_t *)custom_buffer,
                            &size);
        n = write(sockfd, custom_buffer, size);

#endif
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
#endif

#ifdef UDP
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
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                         (struct sockaddr *)&cli_addr, &addr_len);
        buffer[-1] = '\0';
        if (n < 0) {
          continue;
        }

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
          continue;
        }

        power =
            ((std::stoi(extracted.c_str()) + 1) % 50) == 0 ? power + 1 : power;
        std::string test_string(std::pow(10, power), 'B');

        // Send the data back to the server
        std::string msg = "C_" + extracted + "_" + test_string;
        strncpy(buffer, msg.c_str(), sizeof(buffer));
        int msg_len = msg.size();
        n = sendto(sockfd, buffer, msg_len, 0, (struct sockaddr *)&serv_addr,
                   addr_len);
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
#endif

  // Close the socket
  close(sockfd);

  RCLCPP_INFO(this->get_logger(), "Socket closed");
  // Shutdown the node
  rclcpp::shutdown();
}

bool listener::socket_setup() {

#ifdef TCP
  // Create a socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR opening socket");
    return 0;
  }

  if (!socket_tcp::setup_client_socket(sockfd, &serv_addr, port, server_ip)) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: socket setup failed");
    return 0;
  }
#endif

#ifdef UDP
  // Create a socket
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR opening socket");
    return 0;
  }

  if (!socket_udp::sync_check(sockfd, &serv_addr, &cli_addr, port)) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: socket setup failed");
    return 0;
  }
#endif
  return 1;
}
