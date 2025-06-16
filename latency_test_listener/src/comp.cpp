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

  // Terminate session
  close(compfd);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  float a = 1, b = 2, c = 8, d = 9;
  float a_bar, b_bar, c_bar, d_bar;
  int nb_iterations = 10;
  for (int i = 0; i < nb_iterations; i++) {
    a_bar = (a + b) / 2;
    b_bar = (a + b + c) / 3;
    c_bar = (b + c + d) / 3;
    d_bar = (c + d) / 2;
    a = a_bar;
    b = b_bar;
    c = c_bar;
    d = d_bar;
    RCLCPP_INFO(this->get_logger(), "it %d: a = %f | b = %f | c = %f | d = %f",
                i, a, b, c, d);
  }
  float avg = (a + b + c + d) / 4;

  RCLCPP_INFO(this->get_logger(), "EXPECTED AVG for %d iterations: %0.2f",
              nb_iterations, avg);

  RCLCPP_INFO(this->get_logger(), "Shutting down node");
  rclcpp::shutdown();
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
  broad_addr->sin_port = htons(5002);
  // 192.168.0.255 is the broadcast IP for the subnet 192.168.0
  broad_addr->sin_addr.s_addr = INADDR_BROADCAST;

  return true;
}

void comp::enable_socket_write() {
  while (running) {
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
  bool comp_holo_sync = true;
  int end_counter;

  while (running) {
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

        RCLCPP_INFO(this->get_logger(), "Received msg: %s, from %d",
                    msg.c_str(), id);

        // Filter out own broadcast
        if (id == 17) {
          continue;
        }

        if (msg == "READY" && id < 4) {
          holo_ready[id] = true;
        }

        if (msg == "STOP" && id < 4) {
          holo_stop[id] = true;
        }
      }
    }
    if (write_enable || comp_fsm_state == 1 || comp_fsm_state == 3) {
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
      case 3:
        broad_msg = "END";
        end_counter++;
        break;
      default:
        break;
      }

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

    if (comp_fsm_state == START) {
      // Go to sleep for 150 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      comp_fsm_state = STOP;
      continue;
    }

    if (comp_holo_sync) {
      if (holo_ready[0] && holo_ready[1] && holo_ready[2] && holo_ready[3]) {
        comp_fsm_state = START;
        comp_holo_sync = false;
        RCLCPP_INFO(this->get_logger(), "All holohovers ready");
      }
    }

    if (comp_fsm_state == END && end_counter == 10) {
      running = false;
      continue;
    }

    if (holo_stop[0] && holo_stop[1] && holo_stop[2] && holo_stop[3]) {
      comp_fsm_state = END;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
  RCLCPP_INFO(this->get_logger(), "Killing node");
}