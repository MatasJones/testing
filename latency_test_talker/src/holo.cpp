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
  std::thread timer_thread(std::bind(&holo::enable_socket_write, this));
  std::thread sync_thread(std::bind(&holo::holo_holo_sync, this));
  timer_thread.join();
  sync_thread.join();

  RCLCPP_INFO(this->get_logger(), "Starting holo-holo sync");
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
  while (sync_check) {
    write_enable[0] = true;
    write_enable[1] = true;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1000)); // Sleep for the spacing time
  }
  return;
}

void holo::holo_holo_sync() { int a; }

//   char buffer[SYNC_BUFFER_SIZE];
//   // device needs to be sync and it needs to know that neighbour is synced
//   // Needs to keep track of information about the other

//   struct pollfd pfd_1;
//   pfd_1.fd = sockfd[0];
//   pfd_1.events = POLLIN;

//   // Extract the last digit of device and neighbours ip addr
//   char last = this_ip_addrs->device_ip.back();
//   int device_last_ip_digit = last - '0';

//   last = this_ip_addrs->neigh_ip[0].back();
//   int neigh_1_last_ip_digit = last - '0';

//   int neigh_2_last_ip_digit;
//   if (ip_addr.nb_neigh) {
//     last = this_ip_addrs->neigh_ip[1].back();
//     neigh_2_last_ip_digit = last - '0';
//   }

//   struct pollfd pfd_2;
//   pfd_2.fd = sockfd[1];
//   pfd_2.events = POLLIN; // Monitor for incoming data

//   while (sync_check) {
//     // If device and neigh sync checks are true skip next part
//     if (!(sync_validated[0].device && sync_validated[0].neigh)) {
//       // poll socket to see if something needs reading
//       int ret = poll(&pfd_1, 1, 0);
//       if (ret > 0) {
//         if (pfd_1.revents & POLLIN) {
//           RCLCPP_INFO(this->get_logger(), "Received_msg on 1");
//           // Read datagram
//           bzero(buffer, SYNC_BUFFER_SIZE);
//           int n = recvfrom(sockfd[0], buffer, 255, 0,
//                            (struct sockaddr *)&sock_addr[0], &socklen[0]);
//           if (n < 0) {
//             continue;
//           }
//           // Extract flatbuffer payload
//           if (!custom_ser::deser_msg((uint8_t *)buffer, msg, id, value)) {
//             if (++failure_counter > MAX_FAIL_COUNT) {
//               RCLCPP_ERROR(this->get_logger(),
//                            "Too many failures, shutting down");
//               return;
//             }
//             continue;
//           }
//           failure_counter = 0;

//           RCLCPP_INFO(this->get_logger(), "Received_msg: %s", msg.c_str());

//           // Compare msg with expected response
//           std::string expected_response = "ACK_" + ip_addr.neigh_ip[0];
//           if (expected_response == msg) {
//             sync_validated[0] = true;
//             RCLCPP_INFO(this->get_logger(), "Received_msg: %s",
//             msg.c_str()); RCLCPP_INFO(this->get_logger(), "Sync to
//             neighbour 1 check!"); continue;
//           }
//         }
//       }
//       // Write ACK to neighbour 1
//       if (write_enable[0]) {
//         RCLCPP_INFO(this->get_logger(), "Writting sync ACK");
//         std::string this_ACK = "ACK_" + ip_addr.device_ip;
//         custom_ser::ser_msg(this_ACK, 0, 0, &builder, (uint8_t *)&buffer,
//                             &size);
//         strncpy(buffer, this_ACK.c_str(), sizeof(buffer));
//         int n = sendto(sockfd[0], buffer, sizeof(buffer), 0,
//                        (struct sockaddr *)&dest_addr[0],
//                        sizeof(struct sockaddr_in));

//         // Write the data to the socket
//         int n =
//             sendto(sockfd[0], buffer, size, 0, (struct sockaddr
//             *)&dest_addr[0],
//                    sizeof(struct sockaddr_in));
//         if (n < 0) {
//           RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
//           continue;
//         }
//         write_enable[0] = false;
//       }
//     }

// // If there is a second neighbour & sync not validated, perform sync
// if (ip_addr.nb_neigh == 2 && !sync_validated[1]) {
//   // poll socket to see if something needs reading
//   int ret = poll(&pfd_2, 1, 0);
//   if (ret > 0) {
//     if (pfd_2.revents & POLLIN) {
//       RCLCPP_INFO(this->get_logger(), "Received_msg on 2");
//       // Read datagram
//       bzero(buffer, SYNC_BUFFER_SIZE);
//       int n = recvfrom(sockfd[1], buffer, 255, 0,
//                        (struct sockaddr *)&sock_addr[1], &socklen[1]);
//       if (n < 0) {
//         continue;
//       }
//       // Extract flatbuffer payload
//       if (!custom_ser::deser_msg((uint8_t *)buffer, msg, id, value)) {
//         if (++failure_counter > MAX_FAIL_COUNT) {
//           RCLCPP_ERROR(this->get_logger(),
//                        "Too many failures, shutting down");
//           return;
//         }
//         continue;
//       }

//       RCLCPP_INFO(this->get_logger(), "Received_msg: %s", msg.c_str());

//       // Compare msg with expected response
//       std::string expected_response = "ACK_" + ip_addr.neigh_ip[1];
//       if (expected_response == msg) {
//         sync_validated[1] = true;
//         RCLCPP_INFO(this->get_logger(), "Received_msg: %s",
//         msg.c_str()); RCLCPP_INFO(this->get_logger(), "Sync to
//         neighbour 1 check!"); continue;
//       }
//     }
//   }
//   // Write ACK to neighbour 1
//   if (write_enable[1]) {
//     std::string this_ACK = "ACK_" + ip_addr.device_ip;
//     custom_ser::ser_msg(this_ACK, 113, 101, &builder, (uint8_t
//     *)&buffer,
//                         &size);

//     // Write the data to the socket
//     int n = sendto(
//         sockfd[1], buffer, size, 0, (struct sockaddr *)&dest_addr[1],
//         sizeof(struct sockaddr_in)); //  (struct sockaddr *)&dest_addr?
//     if (n < 0) {
//       RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
//       continue;
//     }
//     write_enable[1] = false;
//   }
// }
//   if (sync_validated[0]) { //} && (sync_validated[1] ||
//                            //!(ip_addr.nb_neigh ==
//                            // 2))) {
//     RCLCPP_INFO(this->get_logger(), "Holo sync check!");
//     sync_success = true;
//     return;
//   }
//   std::this_thread::sleep_for(std::chrono::microseconds(10));
// }
//   return;
//   char last = ip_addr.device_ip.back();
//   int ip_last_digit = last - '0';
//   char buffer[1024];
//   RCLCPP_INFO(this->get_logger(), "Last digit: %d", ip_last_digit);
//   // int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

//   if (ip_last_digit == 1) {
//     sockaddr_in sock_addr1;
//     // sock_addr1.sin_family = AF_INET;
//     // sock_addr1.sin_addr.s_addr = INADDR_ANY;
//     // sock_addr1.sin_port = htons(5000);

//     // if (bind(sockfd[0], (struct sockaddr *)&sock_addr1,
//     sizeof(sock_addr1)) <
//     // 0) {
//     //   RCLCPP_INFO(this->get_logger(), "Failed to bind socket");
//     //   close(sockfd[0]);
//     //   return;
//     // }

//     struct pollfd pfd;
//     pfd.fd = sockfd[0];
//     pfd.events = POLLIN;
//     socklen_t socklen1 = sizeof(sock_addr1); // Fixed semicolon

//     while (true) {
//       int ret = poll(&pfd, 1, 0);
//       if (ret > 0) {
//         if (pfd.revents & POLLIN) {
//           RCLCPP_INFO(this->get_logger(), "Received_msg on 1");
//           memset(buffer, 0, sizeof(buffer)); // Use memset instead of bzero
//           int n = recvfrom(sockfd[0], buffer, sizeof(buffer) - 1, 0,
//                            (struct sockaddr *)&sock_addr1, &socklen1);
//           if (n < 0) {
//             continue;
//           }

//           if (!custom_ser::deser_msg((uint8_t *)buffer, msg, id, value)) {
//             if (++failure_counter > MAX_FAIL_COUNT) {
//               RCLCPP_ERROR(this->get_logger(),
//                            "Too many failures, shutting down");
//               close(sockfd[0]);
//               return;
//             }
//             continue;
//           }
//           failure_counter = 0;
//           RCLCPP_INFO(this->get_logger(), "Received_msg: %s", msg.c_str());
//         }
//       }
//       std::this_thread::sleep_for(std::chrono::microseconds(10));
//     }
//   }

//   if (ip_last_digit == 2) {
//     // sockaddr_in serv_addr;
//     //  serv_addr.sin_family = AF_INET;
//     //  serv_addr.sin_addr.s_addr = inet_addr("192.168.0.131");
//     //  serv_addr.sin_port = htons(5002);
//     socklen_t addr_len = sizeof(dest_addr[0]);

//     while (true) {

//       std::string msg = "HeLoLoooo";
//       custom_ser::ser_msg(msg, 0, 0, &builder, (uint8_t *)&buffer, &size);

//       // Write the data to the socket
//       int n = sendto(sockfd[0], buffer, size, 0,
//                      (struct sockaddr *)&dest_addr[0], addr_len);
//       if (n < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
//         continue;
//       }

//       RCLCPP_INFO(this->get_logger(), "MSG sent!");
//       std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Add
//       delay
//     }
//   }

//   close(sockfd[0]); // Clean up
// return;
// }
