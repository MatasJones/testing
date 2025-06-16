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

  RCLCPP_INFO(this->get_logger(), "Starting experiment.");
  std::thread avg_enable_thread(std::bind(&holo::avg_enable, this));
  std::thread exp_thread(std::bind(&holo::perform_exp, this));
  exp_thread.join();
  avg_enable_thread.join();

  // Terminate session
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(this->get_logger(), "Shutting down node");
  rclcpp::shutdown();
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
    dest_addr[n].sin_family = AF_INET;
    dest_addr[n].sin_addr.s_addr = inet_addr(this_ip_addrs.neigh_ip[n].c_str());
    dest_addr[n].sin_port = htons(port);
  }
  return true;
}

void holo::enable_socket_write() {
  while (!sync_success) {
    write_enable[0] = true;
    write_enable[1] = true;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(10)); // Sleep for the spacing time
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
  neigh_1_last_ip_digit = last - '0';

  if (ip_addr.nb_neigh == 2) {
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
          switch ((int)value) {
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
          switch ((int)value) {
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

void holo::avg_enable() {

  while (!averaging) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  while (averaging) {
    write_enable[0] = true;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(5)); // Sleep for the spacing time
  }
  return;
}

void holo::perform_exp() {

  char buffer[SYNC_BUFFER_SIZE];
  // device needs to be sync and it needs to know that neighbour is synced
  // Needs to keep track of information about the other

  struct pollfd pfd_1;
  pfd_1.fd = sockfd[0];
  pfd_1.events = POLLIN;

  struct pollfd pfd_2;
  pfd_2.fd = sockfd[1];
  pfd_2.events = POLLIN;

  // Extract the last digit of device and neighbours ip addr
  char last = ip_addr.device_ip.back();
  int device_last_ip_digit = last - '0';

  last = ip_addr.neigh_ip[0].back();
  neigh_1_last_ip_digit = last - '0';

  if (ip_addr.nb_neigh) {
    last = ip_addr.neigh_ip[1].back();
    neigh_2_last_ip_digit = last - '0';
  }

  if (device_last_ip_digit <= 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "ERROR: could not get device last ip digit");
    running = false;
  }

  // holo_nb: 1 -> 0, 2 -> 1, 8 -> 2, 9 -> 3
  int holo_nb = (device_last_ip_digit < 3) ? (device_last_ip_digit - 1)
                                           : (device_last_ip_digit - 6);

  std::string device_msg;
  int ret, n;
  bool terninate_session = false;

  int iteration_nb = 0;
  // float neigh_1_avg, neigh_2_avg;
  bool new_value_1, new_value_2;
  int nb_participants = (ip_addr.nb_neigh == 2) ? 3 : 2;
  write_enable[0] = false;

  RCLCPP_INFO(this->get_logger(), "Number participants: %d", nb_participants);

  std::deque<float> neigh_1_avg, neigh_2_avg, device_avg;
  neigh_1_avg.push_back(0);
  neigh_2_avg.push_back(0);
  float avg;
  int it;
  bool send_prev_it_1, send_prev_it_2;

  /////// INITIAL AVG VALUE FOR THIS DEVICE ///////
  device_avg.push_back(device_last_ip_digit);
  /////// INITIAL AVG VALUE FOR THIS DEVICE ///////

  // Setup the comp addr for later writting
  memset(&comp_addr, 0, sizeof(comp_addr));
  comp_addr.sin_family = AF_INET; // Means that we are using IPv4
  comp_addr.sin_addr.s_addr = inet_addr("192.168.0.72"); // Set the server IP
  comp_addr.sin_port = htons(COMP_PORT);

  while (running) {
    // poll socket to see if something needs reading
    ret = poll(&pfd_1, 1, 0);
    if (ret > 0) {
      if (pfd_1.revents & POLLIN) {
        // Read datagram
        bzero(buffer, SYNC_BUFFER_SIZE);
        n = recvfrom(sockfd[0], buffer, 255, 0,
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

        // Check if the message is a broadcast
        if (id == 17) {
          if (msg == "READY") {
            RCLCPP_INFO(this->get_logger(), "Comp sent READY msg.");
            device_msg = "READY";
          } else if (msg == "START") {
            RCLCPP_INFO(this->get_logger(), "Comp sent START msg.");
            averaging = true;
            continue;
          } else if (msg == "STOP") {
            RCLCPP_INFO(this->get_logger(), "Comp sent STOP msg.");
            device_msg = "STOP";
            averaging = false;
            write_enable[0] = false;
          } else if (msg == "END") {
            RCLCPP_INFO(this->get_logger(), "Comp sent END msg.");
            terninate_session = true;
            continue;
          } else {
            continue;
          }

          // Send ACK to comp
          custom_ser::ser_msg(device_msg, holo_nb, iteration_nb, &builder,
                              (uint8_t *)&buffer, &size);

          // Write the data to the socket
          n = sendto(sockfd[0], buffer, size, 0, (struct sockaddr *)&comp_addr,
                     sizeof(struct sockaddr_in));
          if (n < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
          }
          continue;
        }

        // Have you received a msg from neighbour 1?
        // If msg == neighbour (holo identification)
        if (msg == ip_addr.neigh_ip[0]) {
          // If my iteration is greater than that of my neighbour, resend
          // previous avg
          if (id < iteration_nb) {
            send_prev_it_1 = true;
          } else {
            // If the difference is larger than a certain amount, it is a new
            // value, push it back
            if (fabs(neigh_1_avg.back() - value) > 0.001f) {
              new_value_1 = true;
              send_prev_it_1 = false;
              averaging = true; // In case message starts before receiving start
                                // from comp
              neigh_1_avg.push_back(value);
            }
          }
        }
      }
    }

    ret = poll(&pfd_2, 1, 0);
    if (ret > 0) {
      if (pfd_2.revents & POLLIN) {
        // Read datagram
        bzero(buffer, SYNC_BUFFER_SIZE);
        n = recvfrom(sockfd[1], buffer, 255, 0,
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

        // Check if the message is a broadcast
        if (id == 17) {
          if (msg == "READY") {
            RCLCPP_INFO(this->get_logger(), "Comp send READY msg.");
            device_msg = "READY";
          } else if (msg == "START") {
            RCLCPP_INFO(this->get_logger(), "Comp send START msg.");
            averaging = true;
            continue;
          } else if (msg == "STOP") {
            RCLCPP_INFO(this->get_logger(), "Comp send STOP msg.");
            device_msg = "STOP";
          } else if (msg == "END") {
            RCLCPP_INFO(this->get_logger(), "Comp send END msg.");
            terninate_session = true;
            continue;
          } else {
            continue;
          }

          // Send ACK to comp
          custom_ser::ser_msg(device_msg, holo_nb, iteration_nb, &builder,
                              (uint8_t *)&buffer, &size);

          // Write the data to the socket
          n = sendto(sockfd[1], buffer, size, 0, (struct sockaddr *)&comp_addr,
                     sizeof(struct sockaddr_in));
          if (n < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
          }
          continue;
        }

        // Do you have a second neighbour, if not skip if statement
        // Have you received a msg from neighbour 2?
        if (ip_addr.nb_neigh == 2 && msg == ip_addr.neigh_ip[1]) {
          if (id < iteration_nb) {
            send_prev_it_2 = true;
          } else {
            // If the difference is smalleer than a certain amount
            if (fabs(neigh_2_avg.back() - value) > 0.001f) {
              new_value_2 = true;
              averaging = true;
              send_prev_it_2 = false;
              neigh_2_avg.push_back(value);
            }
          }
        }
      }
    }

    if (write_enable[0]) {
      if (new_value_1 && (ip_addr.nb_neigh == 1 || new_value_2)) {
        neigh_1_avg.pop_front();
        neigh_2_avg.pop_front();

        avg = (device_avg[iteration_nb] + neigh_1_avg.front() +
               neigh_2_avg.front()) /
              nb_participants;

        device_avg.push_back(avg);

        iteration_nb++;
        new_value_1 = false;
        new_value_2 = false;
      }

      // Write to first neighbour
      it = (send_prev_it_1) ? iteration_nb - 1 : iteration_nb;
      if (it < 0) {
        RCLCPP_ERROR(this->get_logger(), "Tried sending it smaller than 0!");
        continue;
      }

      custom_ser::ser_msg(ip_addr.device_ip, iteration_nb, device_avg[it],
                          &builder, (uint8_t *)&buffer, &size);

      // Write the data to the socket
      n = sendto(sockfd[0], buffer, size, 0, (struct sockaddr *)&dest_addr[0],
                 sizeof(struct sockaddr_in));
      if (n < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to socket avg 1!");
      }

      // If there is a second neigbour, write to it
      if (ip_addr.nb_neigh == 2) {
        it = (send_prev_it_2) ? iteration_nb - 1 : iteration_nb;

        custom_ser::ser_msg(ip_addr.device_ip, iteration_nb, device_avg[it],
                            &builder, (uint8_t *)&buffer, &size);

        n = sendto(sockfd[1], buffer, size, 0, (struct sockaddr *)&dest_addr[1],
                   sizeof(struct sockaddr_in));
        if (n < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error writing to socket avg 2!");
        }
      }
      write_enable[0] = false;
    }

    // Send data to first neighbour

    if (terninate_session) {
      running = false;
      averaging = false;
      RCLCPP_INFO(this->get_logger(), "NB ITERATIONS: %d", iteration_nb);
      RCLCPP_INFO(this->get_logger(), "FINAL AVG: %.02f", device_avg.back());
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}