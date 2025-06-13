#include "comp.h"

comp::comp() : Node("comp") {
  RCLCPP_INFO(this->get_logger(), "Setting up comp.");

  // Setup socket
  comp::device_UDP_socket(&compfd, &sock_addr, &broadcast_addr);
}

bool comp::device_UDP_socket(int *sockfd, struct sockaddr_in *sock_addr,
                             struct sockaddr_in *broad_addr) {

  *sockfd = socket(AF_INET, SOCK_DGRAM, 0); // Create a socket
  if (*sockfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: failed to create socket");
    return false;
  }
  // Setup device socket
  if (!setup_UDP_socket(*sockfd, sock_addr, PORT)) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: failed to start socket");
    return false;
  }

  // Set broadcast parameter
  int use_broadcast = 1;
  memset(broad_addr, 0, sizeof(*broad_addr));
  if (setsockopt(*sockfd, SOL_SOCKET, SO_BROADCAST, &use_broadcast,
                 sizeof(use_broadcast)) < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "ERROR: failed to set broadcast parameter");
  }

  // Create a broadcast addr
  broad_addr->sin_family = AF_INET; // Use IPv4
  broad_addr->sin_port = htons(PORT);
  // 192.168.0.255 is the broadcast IP for the subnet 192.168.0
  inet_pton(AF_INET, "192.168.0.255", &broad_addr->sin_addr);

  return true;
}
