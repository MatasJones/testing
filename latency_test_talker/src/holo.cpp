#include "holo.h"

holo::holo() : Node("holo") {

  RCLCPP_INFO(this->get_logger(), "Starting holo.");

  // Lookup this device's IP address and find neighbouring devices
  RCLCPP_INFO(this->get_logger(), "Looking up device IP and sequence nb...");

  holo::get_ip(&ip_addr);
  RCLCPP_INFO(
      this->get_logger(),
      "Device IP addr: %s, nb neigh: %d, neigh ip 1: %s, neigh ip 2: %s",
      (ip_addr.device_ip).c_str(), ip_addr.nb_neigh,
      (ip_addr.neigh_ip[0]).c_str(), (ip_addr.neigh_ip[1]).c_str());

  // Setup UDP socket to neighbours
  if (holo::device_UDP_socket(ip_addr, sockfd, sock_addr, dest_addr)) {
    // Success - sockfd and sock_addr are now populated
    RCLCPP_INFO(this->get_logger(), "All sockets set up successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Socket setup failed");
  }

  // Perform holo-holo sync check
  RCLCPP_INFO(this->get_logger(), "Starting holo-holo sync");
  std::thread timer_thread(std::bind(&holo::enable_socket_write, this));
  std::thread sync_thread(std::bind(&holo::holo_holo_sync, this));
  timer_thread.join();
  sync_thread.join();

  if (!sync_success) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: failed holo-holo sync check!");
  }
  RCLCPP_INFO(this->get_logger(), "Holo-holo sync check success!");
}

void holo::get_ip(struct ip_addrs *this_ip_addrs) {

  int n;
  struct ifreq ifr;
  char array[] = "wlan0";
  n = socket(AF_INET, SOCK_DGRAM, 0);
  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, array, IFNAMSIZ - 1);
  ioctl(n, SIOCGIFADDR, &ifr);
  close(n);

  this_ip_addrs->device_ip =
      inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);

  char last = this_ip_addrs->device_ip.back();
  int ip_last_digit = last - '0';

  // Holo order left | 131 | 122 | 108 | 109 | right
  switch (ip_last_digit) {
  case 1:
    // Has only 1 neighbour
    this_ip_addrs->nb_neigh = 1;
    this_ip_addrs->neigh_ip[0] = "192.168.0.122"; // PORT: 5002
    break;
  case 2:
    // Has 2 neighbours
    this_ip_addrs->nb_neigh = 2;
    this_ip_addrs->neigh_ip[0] = "192.168.0.131"; // PORT: 5002
    this_ip_addrs->neigh_ip[1] = "192.168.0.108"; // PORT: 5003
    break;
  case 8:
    // Has 2 neighbours
    this_ip_addrs->nb_neigh = 2;
    this_ip_addrs->neigh_ip[0] = "192.168.0.122"; // PORT: 5003
    this_ip_addrs->neigh_ip[1] = "192.168.0.109"; // PORT: 5002
    break;

  case 9:
    // Has only 1 neighbour
    this_ip_addrs->nb_neigh = 1;
    this_ip_addrs->neigh_ip[0] = "192.168.0.108"; // PORT: 5002
    break;
  }
  return;
}

bool holo::device_UDP_socket(struct ip_addrs this_ip_addrs, int sockfd[],
                             struct sockaddr_in sock_addr[],
                             struct sockaddr_in dest_addr[]) {
  for (int n = 0; n < this_ip_addrs.nb_neigh; n++) {
    int port = PORT + n;
    // If holo #3/4, flip ports
    // |holo1|holo2|holo3|holo4|
    // |    2|2   3|3   2|2    | (port: 5000 + ...)
    if (this_ip_addrs.device_ip == "192.168.0.108") {
      RCLCPP_INFO(this->get_logger(), "Switching ports");
      port = PORT + 1 - n;
    }

    sockfd[n] = socket(AF_INET, SOCK_DGRAM, 0); // Create a socket
    if (sockfd[n] < 0) {
      RCLCPP_ERROR(this->get_logger(), "ERROR: failed to create socket");
      return false;
    }
    // Setup device socket
    if (!setup_UDP_socket(sockfd[n], &sock_addr[n], port)) {
      RCLCPP_ERROR(this->get_logger(), "ERROR: failed to start socket for: %s",
                   (this_ip_addrs.neigh_ip[n]).c_str());
      return false;
    }
    // Setup neighbour addr
    memset(&dest_addr[n], 0, sizeof(dest_addr[n]));
    dest_addr[n].sin_family = AF_INET; // Means that we are using IPv4
    dest_addr[n].sin_addr.s_addr =
        inet_addr(this_ip_addrs.neigh_ip[n].c_str()); // Set the server IP
    dest_addr[n].sin_port = htons(port);
  }
  return true;
}

void holo::enable_socket_write() {
  while (!sync_success) {
    write_enable[0] = true;
    write_enable[1] = true;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1000)); // Sleep for the spacing time
  }
  return;
}

void holo::holo_holo_sync() {

  char buffer[SYNC_BUFFER_SIZE];
  // device needs to be sync and it needs to know that neighbour is synced
  // Needs to keep track of information about the other

  struct pollfd pfd_1;
  pfd_1.fd = sockfd[0];
  pfd_1.events = POLLIN;

  // Extract the last digit of device and neighbours ip addr
  char last = ip_addr.device_ip.back();
  int device_last_ip_digit = last - '0';

  last = ip_addr.neigh_ip[0].back();
  int neigh_1_last_ip_digit = last - '0';

  int neigh_2_last_ip_digit;
  if (ip_addr.nb_neigh) {
    last = ip_addr.neigh_ip[1].back();
    neigh_2_last_ip_digit = last - '0';
  }
  std::string this_ACK;

  struct pollfd pfd_2;
  pfd_2.fd = sockfd[1];
  pfd_2.events = POLLIN; // Monitor for incoming data

  while (sync_check) {
    // If device and neigh sync checks are true skip next part
    if (!(sync_validated[0].device && sync_validated[0].neigh)) {
      // poll socket to see if something needs reading
      int ret = poll(&pfd_1, 1, 0);
      if (ret > 0) {
        if (pfd_1.revents & POLLIN) {
          RCLCPP_INFO(this->get_logger(), "Received_msg on 1");
          // Read datagram
          bzero(buffer, SYNC_BUFFER_SIZE);
          int n = recvfrom(sockfd[0], buffer, 255, 0,
                           (struct sockaddr *)&sock_addr[0], &socklen[0]);
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

          RCLCPP_INFO(this->get_logger(), "Received_msg: %s", msg.c_str());

          // If msg is not comming from extected neighbour, ignore it
          if (!(id == neigh_1_last_ip_digit)) {
            continue;
          }

          // If value == 0: ACK request from neighbour
          // If value == 1: ACK response from neighbour
          switch (value) {
          case 0:
            // The neighbour has requested a ACK, send response
            RCLCPP_INFO(this->get_logger(), "Returning ACK to neighbour");
            this_ACK = "ACK_" + ip_addr.device_ip;
            // Set value to 1 to indicate we got the ACK
            custom_ser::ser_msg(this_ACK, device_last_ip_digit, 1, &builder,
                                (uint8_t *)&buffer, &size);

            // Write the data to the socket
            n = sendto(sockfd[0], buffer, size, 0,
                       (struct sockaddr *)&dest_addr[0],
                       sizeof(struct sockaddr_in));
            if (n < 0) {
              RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
            }
            // Note that we can listen to neighbour
            sync_validated[0].neigh = true;
            break;

          case 1:
            // The neighbour has returned our ACK request, we can talk to him,
            // note
            RCLCPP_INFO(this->get_logger(),
                        "Received return ACK from neighbour");
            sync_validated[0].device = true;
            break;

          default:
            break;
          }
        }
      }
      // Write ACK to neighbour 1
      if (write_enable[0]) {
        RCLCPP_INFO(this->get_logger(), "Writting sync ACK");
        this_ACK = "ACK_" + ip_addr.device_ip;
        // value = 0 as we are making an ACK request
        custom_ser::ser_msg(this_ACK, device_last_ip_digit, 0, &builder,
                            (uint8_t *)&buffer, &size);

        // Write the data to the socket
        int n =
            sendto(sockfd[0], buffer, size, 0, (struct sockaddr *)&dest_addr[0],
                   sizeof(struct sockaddr_in));
        if (n < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
          continue;
        }
        write_enable[0] = false;
      }
    }

    // // If there is a second neighbour & sync not validated, perform sync
    if (!(sync_validated[1].device && sync_validated[1].neigh) &&
        (ip_addr.nb_neigh == 2)) {
      // poll socket to see if something needs reading
      int ret = poll(&pfd_2, 1, 0);
      if (ret > 0) {
        if (pfd_2.revents & POLLIN) {
          RCLCPP_INFO(this->get_logger(), "Received_msg on 1");
          // Read datagram
          bzero(buffer, SYNC_BUFFER_SIZE);
          int n = recvfrom(sockfd[1], buffer, 255, 0,
                           (struct sockaddr *)&sock_addr[1], &socklen[1]);
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

          RCLCPP_INFO(this->get_logger(), "Received_msg: %s", msg.c_str());

          // If msg is not comming from extected neighbour, ignore it
          if (!(id == neigh_2_last_ip_digit)) {
            continue;
          }

          // If value == 0: ACK request from neighbour
          // If value == 1: ACK response from neighbour
          switch (value) {
          case 0:
            // The neighbour has requested a ACK, send response
            RCLCPP_INFO(this->get_logger(), "Returning ACK to neighbour");
            this_ACK = "ACK_" + ip_addr.device_ip;
            // Set value to 1 to indicate we got the ACK
            custom_ser::ser_msg(this_ACK, device_last_ip_digit, 1, &builder,
                                (uint8_t *)&buffer, &size);

            // Write the data to the socket
            n = sendto(sockfd[1], buffer, size, 0,
                       (struct sockaddr *)&dest_addr[1],
                       sizeof(struct sockaddr_in));
            if (n < 0) {
              RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
            }
            // Note that we can listen to neighbour
            sync_validated[1].neigh = true;
            break;

          case 1:
            // The neighbour has returned our ACK request, we can talk to him,
            // note
            RCLCPP_INFO(this->get_logger(),
                        "Received return ACK from neighbour");
            sync_validated[1].device = true;
            break;

          default:
            break;
          }
        }
      }
      // Write ACK to neighbour 1
      if (write_enable[1]) {
        RCLCPP_INFO(this->get_logger(), "Writting sync ACK");
        this_ACK = "ACK_" + ip_addr.device_ip;
        // value = 0 as we are making an ACK request
        custom_ser::ser_msg(this_ACK, device_last_ip_digit, 0, &builder,
                            (uint8_t *)&buffer, &size);

        // Write the data to the socket
        int n =
            sendto(sockfd[1], buffer, size, 0, (struct sockaddr *)&dest_addr[1],
                   sizeof(struct sockaddr_in));
        if (n < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
          continue;
        }
        write_enable[1] = false;
      }
    }

    // If all checks are completed, leave thread
    if ((sync_validated[0].device && sync_validated[0].neigh) &&
        ((sync_validated[1].device && sync_validated[1].neigh) ||
         !(ip_addr.nb_neigh == 2))) {
      RCLCPP_INFO(this->get_logger(), "Holo sync check!");
      sync_success = true;
      return;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
}