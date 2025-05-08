#ifndef LISTENER_H
#define LISTENER_H

#include "custom_msg/msg/custom_string.hpp"
#include "custom_msg/msg/int16msg.hpp"
#include "custom_msg/msg/sync_msg.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sync_service/srv/sync_check.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <netdb.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h> // gethostname
#include <yaml-cpp/yaml.h>

#define QUEUE_SIZE 100
#define SOCKET_BUFFER_SIZE 65490

class listener : public rclcpp::Node {

public:
  listener();
  int count_ = 0;

private:
  // Declare a Publisher
  rclcpp::Publisher<custom_msg::msg::CustomString>::SharedPtr
      listener_publisher_;

  rclcpp::Publisher<custom_msg::msg::SyncMsg>::SharedPtr sync_publisher_;

  // Declare a subscriber
  rclcpp::Subscription<custom_msg::msg::CustomString>::SharedPtr
      listener_subscriber_;

  rclcpp::Subscription<custom_msg::msg::SyncMsg>::SharedPtr sync_subscriber_;

  void echo(const custom_msg::msg::CustomString::SharedPtr msg);
  void echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg);
  void get_ip_addr();
  bool socket_setup();

  std::string ip_addr;
  std::filesystem::path cwd = std::filesystem::current_path();
  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  int id;
  int port = 5000;
  char server_ip[13] = "192.168.0.72"; // Server IP
  int sockfd;
  int power = 0;

  // UDP
  struct sockaddr_in *serv_addr;

  bool running = true;
};
#endif // LISTENER_H

#ifdef UDP
// Setup UDP socket
sockfd = socket(AF_INET, SOCK_DGRAM, 0);
if (sockfd < 0) {
  RCLCPP_ERROR(this->get_logger(), "ERROR opening socket");
  return 0;
}

// Allocate memory for serv_addr pointer (since it's declared in the header)
serv_addr = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
if (serv_addr == NULL) {
  RCLCPP_ERROR(this->get_logger(), "ERROR allocating memory for serv_addr");
  close(sockfd);
  return 0;
}

// Reset all the serv_addr values to zero
memset(serv_addr, 0, sizeof(struct sockaddr_in)); // Use memset instead of bzero
                                                  // for better portability

// Set the serv_addr parameters
serv_addr->sin_family = AF_INET; // Means that we are using IPv4
serv_addr->sin_addr.s_addr = inet_addr(
    "127.0.0.1"); // Set to server IP address (replace with actual server IP)
serv_addr->sin_port =
    htons(port); // Convert port from host byte order to network byte order

// UDP client does not need to bind to a specific port, but if you need to
// receive responses you may need to do so

// Optional: If you need to bind to receive responses
struct sockaddr_in client_addr;
memset(&client_addr, 0, sizeof(client_addr));
client_addr.sin_family = AF_INET;
client_addr.sin_addr.s_addr = INADDR_ANY; // Listen on any interface
client_addr.sin_port = 0;                 // Let the OS assign a port

// Optionally bind the socket if you need to receive on a specific interface
// bind(sockfd, (struct sockaddr*)&client_addr, sizeof(client_addr));

// Send a message to the server
int sync_msg = sendto(sockfd, "First msg", 10, 0, (struct sockaddr *)serv_addr,
                      sizeof(struct sockaddr_in));
if (sync_msg < 0) {
  RCLCPP_ERROR(this->get_logger(), "ERROR sending message");
  free(serv_addr);
  serv_addr = NULL;
  close(sockfd);
  return 0;
}

// Wait for a response from the server
char buffer[256]; // Define buffer size
socklen_t serv_len =
    sizeof(struct sockaddr_in); // Need a socklen_t variable for recvfrom

sync_msg =
    recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)serv_addr, &serv_len);
if (sync_msg < 0) {
  RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
  free(serv_addr);
  serv_addr = NULL;
  close(sockfd);
  return 0;
}

buffer[sync_msg] = '\0'; // Null-terminate the received data

if (strncmp(buffer, "SERVER_ACK", 10) != 0) {
  RCLCPP_ERROR(this->get_logger(), "ERROR: server did not acknowledge");
  free(serv_addr);
  serv_addr = NULL;
  close(sockfd);
  return 0;
}

RCLCPP_INFO(this->get_logger(), "Message from server: %s", buffer);

// Send acknowledgment to the server
sync_msg = sendto(sockfd, "CLIENT_ACK", 10, 0, (struct sockaddr *)serv_addr,
                  sizeof(struct sockaddr_in));
if (sync_msg < 0) {
  RCLCPP_ERROR(this->get_logger(), "ERROR sending acknowledgment");
  free(serv_addr);
  serv_addr = NULL;
  close(sockfd);
  return 0;
}

RCLCPP_INFO(this->get_logger(), "UDP socket setup done");
#endif