#include "listener.h"

// #define UDP
//  #define TCP
#define RAW
#define CUSTOM_ETHERTYPE 0x88B5

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
        RCLCPP_INFO(this->get_logger(), "received message: %s", buffer);
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

        // Send response to server
        if (!custom_ser::deser_msg((uint8_t *)buffer, msg, id, value)) {
          if (failure_counter++ > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            running = false;
          }
          continue;
        }
        failure_counter = 0;

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
        if (n < 0) {
          continue;
        }

        if (!custom_ser::deser_msg((uint8_t *)buffer, msg, id, value)) {
          if (failure_counter++ > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            running = false;
          }
          continue;
        }
        failure_counter = 0;

        if (msg == "SHUTDOWN") {
          RCLCPP_INFO(this->get_logger(), "Shutdown message received");
          running = false;
          std::this_thread::sleep_for(std::chrono::seconds(5));
          continue;

        } else if (msg == "GRACE") {
          // RCLCPP_INFO(this->get_logger(), "Grace message received");
          msg = "GRACE_ACK";
        }

        else {
          std::string test_string(std::pow(10, value), 'B');
          msg = test_string;
        }

        custom_ser::ser_msg(msg, id, value, &builder, (uint8_t *)buffer, &size);

        n = sendto(sockfd, buffer, size, 0, (struct sockaddr *)&serv_addr,
                   addr_len);
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
#endif

#ifdef RAW

  // Create a poll to verify if the socket is ready to read
  struct pollfd fds;
  fds.fd = sockfd;
  fds.events = POLLIN; // Check for incoming data

  char raw_buffer[1524];
  char frame[1524];
  struct ethhdr *eth_read = (struct ethhdr *)raw_buffer;
  int n;

  struct ethhdr *eth_write = (struct ethhdr *)frame;
  memcpy(eth_write->h_dest, MAC_131, 6);
  memcpy(eth_write->h_source, MAC_122, 6);
  eth_write->h_proto = htons(CUSTOM_ETHERTYPE);

  size_t frame_len, sent;
  char *payload;

  while (running) {
    int ret = poll(&fds, 1, 0); // Determine the state of the socket
    // 1) Read incoming data from the socket
    if (ret > 0) {
      if (fds.revents & POLLIN) {

        // Read the data from the socket
        n = recv(sockfd, raw_buffer, sizeof(raw_buffer), 0);

        if (n < sizeof(struct ethhdr)) {
          continue;
        }
        if (ntohs(eth_read->h_proto) != CUSTOM_ETHERTYPE) {
          continue; // Skip non-custom frames
        }
        // Correct MAC address comparison using memcmp()
        if (memcmp(eth_read->h_dest, MAC_122, 6) != 0 ||
            memcmp(eth_read->h_source, MAC_131, 6) != 0) {
          continue; // Skip frames not intended for us
        }

        if (raw_buffer[14] == 0x53) {
          continue;
        }

        payload = raw_buffer + sizeof(struct ethhdr);
        if (!custom_ser::deser_msg((uint8_t *)payload, msg, id, value)) {
          // RCLCPP_ERROR(this->get_logger(),
          //              "Error deserializing flatbuffer message!");
          if (failure_counter++ > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            running = false;
          }
          continue;
        }
        failure_counter = 0;

        if (msg == "SHUTDOWN") {
          RCLCPP_INFO(this->get_logger(), "Shutdown message received");
          running = false;
          std::this_thread::sleep_for(std::chrono::seconds(5));
          continue;

        } else if (msg == "GRACE") {
          msg = "GRACE_ACK";
        }

        else {
          std::string test_string(std::pow(10, value), 'B');
          msg = test_string;
        }

        custom_ser::ser_msg(msg, id, value, &builder, (uint8_t *)raw_buffer,
                            &size);

        // Add payload after ethernet header
        memcpy(frame + sizeof(struct ethhdr), raw_buffer, size);

        frame_len = sizeof(struct ethhdr) + size;
        // Ensure minimum frame size (64 bytes total)
        // Minimum size for ethernet frames is 64 bytes
        if (frame_len < 64) {
          memset(frame + sizeof(struct ethhdr) + size, 0, 64 - frame_len);
          frame_len = 64;
        }

        sent = sendto(sockfd, frame, frame_len, 0, (struct sockaddr *)&sll,
                      sizeof(sll));
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
  RCLCPP_INFO(this->get_logger(), "Device UDP socketfd: %d", sockfd);

  if (!socket_udp::sync_check(sockfd, &serv_addr, &cli_addr, port)) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: socket setup failed");
    return 0;
  }
#endif

#ifdef RAW
  // Create a socket
  /*
  AF_PACKET: data link layer
  SOCK_RAW: raw socket
  htons(ETH_P_ALL): allows us to craft our own header
  */
  sockfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));

  // // Bind the socket to the WIFI card interface
  if (setup_raw_socket(&sockfd, &sll, MAC_131) == 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: socket setup failed");
    return 0;
  }

  // Check RAW sync
  raw_sync_check(sockfd, sll, MAC_131, MAC_122);
  RCLCPP_INFO(this->get_logger(), "Sync done");

#endif
  return 1;
}
