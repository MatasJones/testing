#include "comp.h"

comp::comp() : Node("comp") {
  RCLCPP_INFO(this->get_logger(), "Setting up comp.");

  // Setup socket
  if (!comp::device_UDP_socket(&compfd, &sock_addr, &broadcast_addr)) {
    RCLCPP_ERROR(this->get_logger(),
                 "ERROR: failed to create broadcast socket");
  }
  RCLCPP_INFO(this->get_logger(), "Broadcast socket setup.");

  std::thread timer_thread(std::bind(&comp::enable_socket_write, this));
  std::thread start_thread(std::bind(&comp::exp_start, this));
  timer_thread.join();
  start_thread.join();
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
    return false;
  }

  // Create a broadcast addr
  broad_addr->sin_family = AF_INET; // Use IPv4
  broad_addr->sin_port = htons(PORT);
  // 192.168.0.255 is the broadcast IP for the subnet 192.168.0
  inet_pton(AF_INET, "192.168.0.255", &broad_addr->sin_addr);

  return true;
}

void comp::enable_socket_write() {
  while (!start_success) {
    write_enable = true;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)); // Sleep for the spacing time
  }
  return;
}

void comp::exp_start() {
  char buffer[SYNC_BUFFER_SIZE];
  // device needs to be sync and it needs to know that neighbour is synced
  // Needs to keep track of information about the other

  struct pollfd pfd;
  pfd.fd = compfd;
  pfd.events = POLLIN;

  std::string broad_msg;

  while (!start_success) {
    int ret = poll(&pfd, 1, 0);
    // If there is something to read, do it
    if (ret > 0) {
      if (pfd.revents & POLLIN) {
        // Read datagram
        bzero(buffer, SYNC_BUFFER_SIZE);
        int n = recvfrom(compfd, buffer, 255, 0, (struct sockaddr *)&sock_addr,
                         &socklen);
        if (n < 0) {
          continue;
        }
        // Extract flatbuffer payload
        if (!custom_ser::deser_msg((uint8_t *)buffer, msg, id, value)) {
          if (++failure_counter > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            return;
          }
          continue;
        }
        failure_counter = 0;

        // Filter out own broadcast
        if (id == 17) {
          continue;
        }

        RCLCPP_INFO(this->get_logger(), "Received_msg: %s", msg.c_str());
      }
    }
    if (write_enable) {
      switch (comp_fsm_state) {
      case 0:
        broad_msg = "READY";
        break;
      case 1:
        broad_msg = "START";
        break;
      case 2:
        broad_msg = "STOP";
        break;
      default:
        break;
      }

      RCLCPP_INFO(this->get_logger(), "Broadcasting: %s", broad_msg.c_str());

      custom_ser::ser_msg(broad_msg, 17, 0, &builder, (uint8_t *)&buffer,
                          &size);

      // Write the data to the socket
      int n =
          sendto(compfd, buffer, size, 0, (struct sockaddr *)&broadcast_addr,
                 sizeof(struct sockaddr_in));
      if (n < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
        continue;
      }
      write_enable = false;
    }
  }
}