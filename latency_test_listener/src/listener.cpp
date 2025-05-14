#include "listener.h"

#define UDP
// #define TCP

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
  RCLCPP_INFO(this->get_logger(), "Let's rumble!");

#ifdef TCP
  // Create a poll to verify if the socket is ready to read
  struct pollfd fds;
  fds.fd = sockfd;
  fds.events = POLLIN; // Check for incoming data

  char buffer[SOCKET_BUFFER_SIZE];

  while (running) {
    int ret = poll(&fds, 1, 0); // Determine the state of the socket
    // 1) Read incoming data from the socket
    if (ret > 0) {
      if (fds.revents & POLLIN) {

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
          // RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
          continue;
        }
        // RCLCPP_INFO(this->get_logger(), "%s\n", buffer);
        //  Send back data straight away

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
        RCLCPP_INFO(this->get_logger(), "Extracted msg number: %s",
                    extracted.c_str());
        power =
            ((std::stoi(extracted.c_str()) + 1) % 50) == 0 ? power + 1 : power;
        std::string test_string(std::pow(10, power), 'B');

        // Send the data back to the server
        std::string msg = "C_" + extracted + "_" + test_string;
        RCLCPP_INFO(this->get_logger(), "Sending response: %s", msg.c_str());
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
  // Variable setup
  int n;
  // struct hostent *server;
  char buffer[256];
  // Setup UDP socket
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR opening socket");
    return 0;
  }

  // Reset all the serv_addr values to zero
  // bzero((char *)&serv_addr, sizeof(serv_addr)); // The function bzero() sets
  // all
  // values in a buffer to zero

  // Set the serv_addr parameters
  serv_addr.sin_family = AF_INET; // Means that we are using IPv4
  // Tell the socket to accept any of the host machines IPs
  serv_addr.sin_addr.s_addr = inet_addr("192.168.0.72"); // Set the server IP
  serv_addr.sin_port =
      htons(port); // This sets the port number the server will listen on,
                   // converting it from host byte order to network byte order

  bzero(&cli_addr, sizeof(cli_addr));
  cli_addr.sin_family = AF_INET;
  cli_addr.sin_addr.s_addr =
      INADDR_ANY; // This means bind to any available network interface
  cli_addr.sin_port = htons(port);

  if (bind(sockfd, (struct sockaddr *)&cli_addr, sizeof(cli_addr)) < 0) {
    RCLCPP_INFO(this->get_logger(), "ERROR on binding");
  }

  // UDP client does not need to know its own port, it is handled by the OS as
  // there is no full connection like TCP
  RCLCPP_INFO(this->get_logger(),
              "Created socket, sending message to server...");
  // Send a message to the server
  int sync_msg = sendto(sockfd, "First msg", 9, 0,
                        (struct sockaddr *)&serv_addr, addr_len);
  if (sync_msg < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR sending message");
    return 0;
  }
  RCLCPP_INFO(this->get_logger(),
              "Message sent to server, waiting for response...");

  /*
  ssize_t recvfrom(int sockfd, void *buf, size_t len, int flags,
                   struct sockaddr *src_addr, socklen_t *addrlen);
  */

  // Wait for a response from the server
  sync_msg = recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)&cli_addr,
                      &addr_len); // n is the number of bytes read
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
  sync_msg = sendto(sockfd, "CLIENT_ACK", 10, 0, (struct sockaddr *)&cli_addr,
                    addr_len);

  RCLCPP_INFO(this->get_logger(), "UDP socket setup done");

#endif
  return 1;
}
