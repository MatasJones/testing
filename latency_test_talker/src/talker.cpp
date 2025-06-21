#include "talker.h"

//#define TCP
//#define UDP
#define RAW

// #define MANUAL_SER
#define FLATBUFF_SER

talker::talker() : Node("talker") {

  RCLCPP_INFO(this->get_logger(), "Creating talker");

  // Declare variables from the given ones during the launch
  this->declare_parameter("spacing_ms",
                          DEAFULT_SPACING); // Default value of 400ms
  spacing_ms_ = this->get_parameter("spacing_ms")
                    .as_int(); // Set the spacing as the value passed or as the
                               // default value

  this->declare_parameter("msg_size", DEFAULT_MSG_SIZE);
  msg_size = this->get_parameter("msg_size").as_int();

#ifdef RAW
  msg_size = 4;
#endif

  total_nb_msgs = msg_size * NB_MSGS;

  RCLCPP_INFO(this->get_logger(), "Spacing time %d ms", spacing_ms_);

  RCLCPP_INFO(this->get_logger(), "Msg size %d", msg_size);

  RCLCPP_INFO(this->get_logger(), "Total size %d", total_nb_msgs);

  // Setup the logger for later use
  talker::create_logger();

  // Set up the experiment parameters from the config file
  talker::setup_experiment();

  // Create a socket and bind it to the server port and ip
  RCLCPP_INFO(this->get_logger(), "Creating socket...");
  if (!talker::socket_setup()) {
    RCLCPP_INFO(this->get_logger(), "Socket setup failed");
    rclcpp::shutdown();

  } else {
    RCLCPP_INFO(this->get_logger(), "Socket setup done");
  }

  RCLCPP_INFO(this->get_logger(), "Client IP: %s",
              inet_ntoa(cli_addr.sin_addr));
  RCLCPP_INFO(this->get_logger(), "Client Port: %d", ntohs(cli_addr.sin_port));

#ifdef FLATBUFF_SER
  RCLCPP_INFO(this->get_logger(), "Setting up flatbuffers...");
  // Create the flatbuffer message
  char test_buffer[SOCKET_BUFFER_SIZE];
  custom_ser::ser_msg("TEST", 1, 42, &builder, (uint8_t *)test_buffer, &size);
  // Deserialize the flatbuffer message
  if (!custom_ser::deser_msg((uint8_t *)test_buffer, msg, id, value)) {
    RCLCPP_ERROR(this->get_logger(), "Error deserializing flatbuffer message");
  } else {
    if (msg == "TEST" && id == 1 && value == 42) {
      RCLCPP_INFO(this->get_logger(), "Flatbuffer working");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "ERROR DURING FLATBUFFER INITIALIZATION");
    }
  }

  // custom_seri = true;

#endif

  RCLCPP_INFO(this->get_logger(), "Let's rumble!");

  // This thread is called perdiodically to enable writing to the socket every
  // spacing_ms
  std::thread timer_thread(std::bind(&talker::enable_socket_write, this));

  // Start experiment thread
  std::thread exp_thread(std::bind(&talker::socket_exp_launch, this));

  exp_thread.join();
  timer_thread.join();

  // Finish the experiment
  RCLCPP_INFO(this->get_logger(), "Experiment finished, closing socket...");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  talker::terminate_exp();
}

void talker::enable_socket_write() {
  while (running) {
    write_enable = true;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(spacing_ms_)); // Sleep for the spacing time
  }
  return;
}

void talker::socket_exp_launch() {
  // Create a timer to enable writing to socket periodically
  this->socket_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(spacing_ms_),
                              std::bind(&talker::enable_socket_write, this));
#ifdef TCP
  // Create a poll to monitor the socket
  struct pollfd pfd;
  pfd.fd = server_sockfd;
  pfd.events = POLLIN; // Monitor for incoming data

  char buffer[SOCKET_BUFFER_SIZE];
  char custom_buffer[1000000];

  // This while loop take ~60µs to execute
  while (running) {
    int ret = poll(&pfd, 1, 0);
    // 1) Check it there is any data that needs to be read on the server socket
    if (ret > 0) {
      if (pfd.revents & POLLIN) {

        // Read the data from the socket
        bzero(custom_buffer, SOCKET_BUFFER_SIZE);
        int n = read(server_sockfd, custom_buffer, SOCKET_BUFFER_SIZE - 1);
        // If there was an issue when reading the socket, skip the iteration
        if (n < 0) {
          break;
        }

        // Set timestamp
        double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;

// MANUAL SERIALIZATION
#ifdef MANUAL_SER
        // Verify message is valid
        std::string message_id = socket_tcp::extract_message(custom_buffer);
        if (message_id == "") {
          continue;
        }

        // Grace period counter
        if (std::stoi(message_id.c_str()) == 131313) {
          grace_counter_read++;
          continue;
        }

        // Add the time to the socket_send_receive_time array
        std::get<1>(socket_send_receive_time[atoi(message_id.c_str())]) =
            recieving_time;

        // Add msg id to the socket_send_receive_time array
        std::get<3>(socket_send_receive_time[atoi(message_id.c_str())]) =
            atoi(message_id.c_str());
#endif

#ifdef FLATBUFF_SER
        if (!custom_ser::deser_msg((uint8_t *)custom_buffer, msg, id, value)) {
          // RCLCPP_ERROR(this->get_logger(),
          //              "Error deserializing flatbuffer message!");
          if (failure_counter > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            running = false;
            break;
          }
          continue;
        }
        if (msg == "GRACE_ACK") {
          RCLCPP_INFO(this->get_logger(), "Grace message received");
          grace_counter_read++;
          continue;
        }

        // Add the time to the socket_send_receive_time array
        std::get<1>(socket_send_receive_time[id]) = recieving_time;

        // Add msg id to the socket_send_receive_time array
        std::get<3>(socket_send_receive_time[id]) = id;
#endif
      }
    }
    // 2) Send the data to the server
    if (write_enable && ret == 0) {
      // If package size is reached, increase the size
      if ((socket_msg_count + 1) % NB_MSGS == 0) {
        socket_msg_size++;
      }
#ifdef FLATBUFF_SER

      // If grace period
      if (grace == true) {
        // Send grace message and count the number of messages sent
        if (!socket_tcp::grace_writer(server_sockfd, &grace_counter_write,
                                      grace_counter_read, &grace, true)) {
          RCLCPP_ERROR(this->get_logger(), "Error writing grace message!");
        }

        write_enable = false;
        continue;
      }
#endif

#ifdef MANUAL_SER

      // If grace period
      if (grace == true) {
        // Send grace message and count the number of messages sent
        socket_tcp::grace_writer(server_sockfd, &grace_counter_write,
                                 grace_counter_read, &grace, false);
        write_enable = false;
        continue;
      }

      // Write the data to the socket
      bzero(buffer, SOCKET_BUFFER_SIZE);
      std::string test_string(sizes[socket_msg_size], 'A');

      // If not grace period
      std::string msg =
          "S_" + std::to_string(socket_msg_count) + "_" + test_string;
      strncpy(buffer, msg.c_str(), sizeof(buffer));

      int n = write(server_sockfd, buffer, strlen(buffer));
      if (n < 0) {
        continue;
      }
#endif

#ifdef FLATBUFF_SER
      std::string test_string(sizes[socket_msg_size], 'A');
      custom_ser::ser_msg(test_string, socket_msg_count, socket_msg_size,
                          &builder, (uint8_t *)&custom_buffer, &size);

      // Write the data to the socket
      int n = write(server_sockfd, custom_buffer, size);
      if (n < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to socket!");
        continue;
      }
#endif

      double sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;
      // Add the time to the socket_send_receive_time array
      std::get<0>(socket_send_receive_time[socket_msg_count]) = sending_time;
      // Add msg id to the socket_send_receive_time array
      std::get<2>(socket_send_receive_time[socket_msg_count]) =
          socket_msg_count;

      // Increment the message count
      socket_msg_count++;

      // If there is no data to be read nor to write, terminate session
      if (socket_msg_count > total_nb_msgs) {
        RCLCPP_INFO(this->get_logger(), "Socket session terminated");
        running = false;
        std::this_thread::sleep_for(std::chrono::seconds(3));
      }

      write_enable = false; // Disable writing until next timer event
    }

    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
#endif

#ifndef TCP

  // Create a poll to monitor the socket
  struct pollfd pfd;
  pfd.fd = sockfd;
  pfd.events = POLLIN; // Monitor for incoming data

  char buffer[SOCKET_BUFFER_SIZE];
  char raw_buffer[1514];
  struct ethhdr *eth_read = (struct ethhdr *)raw_buffer;
  int n;

  char frame[1524];
  struct ethhdr *eth_write = (struct ethhdr *)frame;
  memcpy(eth_write->h_dest, MAC_122, 6);
  memcpy(eth_write->h_source, MAC_131, 6);
  eth_write->h_proto = htons(CUSTOM_ETHERTYPE);

  size_t frame_len, sent;
  int grace_counter_write, grace_counter_read;

  RCLCPP_INFO(this->get_logger(), "Starting exp");

  // This while loop take ~60µs to execute
  while (running) {
    int ret = poll(&pfd, 1, 0);
    // 1) Check it there is any data that needs to be read on the server socket
    if (ret > 0) {
      if (pfd.revents & POLLIN) {
        // Read the data from the socket
#ifdef UDP
        bzero(buffer, SOCKET_BUFFER_SIZE);
        int n = recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)&serv_addr,
                         &clilen);

        if (n < 0) {
          continue;
        }

        // Set timestamp
        double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;

        if (!custom_ser::deser_msg((uint8_t *)buffer, msg, id, value)) {
          // RCLCPP_ERROR(this->get_logger(),
          //              "Error deserializing flatbuffer message!");
          if (failure_counter > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            running = false;
            break;
          }
          continue;
        }
        if (msg == "GRACE_ACK") {
          RCLCPP_INFO(this->get_logger(), "Grace message received");
          grace_counter_read++;
          continue;
        }

        // Add the time to the socket_send_receive_time array
        std::get<1>(socket_send_receive_time[id]) = recieving_time;

        // Add msg id to the socket_send_receive_time array
        std::get<3>(socket_send_receive_time[id]) = id;
#endif

#ifdef RAW
        // Read interface and verfy that msg is for us
        n = recv(sockfd, raw_buffer, sizeof(raw_buffer), 0);
        if (n < 1) {
          continue;
        }
        // Set timestamp
        double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;
        // Check if frame is large enough to contain Ethernet header
        if (n < sizeof(struct ethhdr)) {
          continue;
        }
        if (ntohs(eth_read->h_proto) != CUSTOM_ETHERTYPE) {
          continue; // Skip non-custom frames
        }
        // Correct MAC address comparison using memcmp()
        if (memcmp(eth_read->h_dest, MAC_131, 6) != 0 ||
            memcmp(eth_read->h_source, MAC_122, 6) != 0) {
          continue; // Skip frames not intended for us
        }
        // If we got this far, means message is for us, read payload
        // Read payload
        char *payload = raw_buffer + sizeof(struct ethhdr);
        if (!custom_ser::deser_msg((uint8_t *)payload, msg, id, value)) {
          // RCLCPP_ERROR(this->get_logger(),
          //              "Error deserializing flatbuffer message!");
          if (failure_counter++ > MAX_FAIL_COUNT) {
            RCLCPP_ERROR(this->get_logger(),
                         "Too many failures, shutting down");
            running = false;
            break;
          }
          continue;
        }
        if (msg == "GRACE_ACK" && grace == true) {
          RCLCPP_INFO(this->get_logger(), "Grace message nb: %d", id);
          grace_counter_read++;
          continue;
        }

        // Add the time to the socket_send_receive_time array
        std::get<1>(socket_send_receive_time[id]) = recieving_time;

        // Add msg id to the socket_send_receive_time array
        std::get<3>(socket_send_receive_time[id]) = id;

#endif
      }
    }
    // 2) Send the data
    if (write_enable && ret == 0) {
      if ((socket_msg_count + 1) % NB_MSGS == 0) {
        socket_msg_size++;
      }

#ifdef UDP
      if (grace == true) {
        if (grace_counter_write > GRACE_COUNTER_MAX &&
            grace_counter_read > GRACE_COUNTER_MAX) {
          grace = false;
          RCLCPP_INFO(this->get_logger(),
                      "Grace period ended, starting experiment");
          continue;
        };

        custom_ser::ser_msg("GRACE", grace_counter_write, 404, &builder,
                            (uint8_t *)buffer, &size);

        sent = sendto(sockfd, buffer, size, 0, (struct sockaddr *)&dest_addr,
                      sizeof(struct sockaddr_in));
        if (sent > 0) {
          grace_counter_write++;
        }

        write_enable = false;
        continue;
      }

      std::string test_string(sizes[socket_msg_size], 'A');
      custom_ser::ser_msg(test_string, socket_msg_count, socket_msg_size,
                          &builder, (uint8_t *)&buffer, &size);

      double sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;

      // Write the data to the socket
      sent = sendto(sockfd, buffer, size, 0, (struct sockaddr *)&dest_addr,
                    sizeof(struct sockaddr_in));
      if (sent < 0) {
        RCLCPP_ERROR(this->get_logger(), "ERROR whilst sending test msg");
      }

      // Add the time to the socket_send_receive_time array
      std::get<0>(socket_send_receive_time[socket_msg_count]) = sending_time;
      // Add msg id to the socket_send_receive_time array
      std::get<2>(socket_send_receive_time[socket_msg_count]) =
          socket_msg_count;

      // Increment the message count
      socket_msg_count++;

      // If there is no data to be read nor to write, terminate session
      if (socket_msg_count >= total_nb_msgs - 1) {
        RCLCPP_INFO(this->get_logger(), "Socket session terminated");
        running = false;
        std::this_thread::sleep_for(std::chrono::seconds(3));
      }

      write_enable = false; // Disable writing until next timer event

#endif

#ifdef RAW
      if (grace == true) {
        if (grace_counter_write > GRACE_COUNTER_MAX &&
            grace_counter_read > GRACE_COUNTER_MAX) {
          grace = false;
          RCLCPP_INFO(this->get_logger(),
                      "Grace period ended, starting experiment");
          continue;
        }

        custom_ser::ser_msg("GRACE", grace_counter_write, 404, &builder,
                            (uint8_t *)raw_buffer, &size);

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
        if (sent > 0) {
          grace_counter_write++;
        }

        write_enable = false;
        continue;
      }

      std::string test_string(sizes[socket_msg_size], 'A');
      custom_ser::ser_msg(test_string, socket_msg_count, socket_msg_size,
                          &builder, (uint8_t *)&raw_buffer, &size);

      // Add payload after ethernet header
      memcpy(frame + sizeof(struct ethhdr), raw_buffer, size);

      frame_len = sizeof(struct ethhdr) + size;
      // Ensure minimum frame size (64 bytes total)
      // Minimum size for ethernet frames is 64 bytes
      if (frame_len < 64) {
        memset(frame + sizeof(struct ethhdr) + size, 0, 64 - frame_len);
        frame_len = 64;
      }

      // Write the data to the socket
      sent = sendto(sockfd, frame, frame_len, 0, (struct sockaddr *)&sll,
                    sizeof(sll));
      if (sent < 0) {
        RCLCPP_ERROR(this->get_logger(), "ERROR whilst sending test msg");
      }

      double sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;
      // Add the time to the socket_send_receive_time array
      std::get<0>(socket_send_receive_time[socket_msg_count]) = sending_time;
      // Add msg id to the socket_send_receive_time array
      std::get<2>(socket_send_receive_time[socket_msg_count]) =
          socket_msg_count;

      // Increment the message count
      socket_msg_count++;

      // If there is no data to be read nor to write, terminate session
      if (socket_msg_count > total_nb_msgs - 1) {
        RCLCPP_INFO(this->get_logger(), "Socket session terminated");
        running = false;
        std::this_thread::sleep_for(std::chrono::seconds(3));
      }

      write_enable = false; // Disable writing until next timer event

#endif
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

#endif
}

void talker::create_logger() {

  this->file.open(this->logger_name, std::ios::out | std::ios::trunc);

  if (!this->file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Error occurred during file creation!");
    return;
  }
  string file_name = std::to_string(spacing_ms_) + "ms";
  this->file << file_name << "\n";

  this->file.close();
  RCLCPP_INFO(this->get_logger(), "Logger created");
}

template <typename T> void talker::log(T data) {
  this->file.open(this->logger_name, std::ios::out | std::ios::app);
  if (!this->file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Error occurred during file writing!");
    return;
  }
  this->file << data << "\n";
  this->file.close();
}

void talker::setup_experiment() {

  YAML::Node config = YAML::LoadFile(config_file_path); // Load the config file
  repetitions =
      config["repetitions"].as<int>(); // Exract the experiment parameters
  sizes = config["sizes"].as<std::vector<int>>();
}

void talker::terminate_exp() {
  RCLCPP_INFO(this->get_logger(), "Experiment completed");
  // Tell listener nodes to shutdown
  char buffer[256];
#ifdef TCP

#ifdef MANUAL_SER
  std::string terminate_str = "SHUTDOWN";
  strncpy(buffer, terminate_str.c_str(), sizeof(buffer));
  int n = write(server_sockfd, buffer, strlen(buffer));
#endif

#ifdef FLATBUFF_SER
  RCLCPP_INFO(this->get_logger(), "Sending shutdown message");
  custom_ser::ser_msg("SHUTDOWN", 13, 404, &builder, (uint8_t *)buffer, &size);
  int n = write(server_sockfd, buffer, size);
  std::this_thread::sleep_for(std::chrono::seconds(1));
#endif

  // Close the socket
  if (close(server_sockfd) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error closing socket");
  } else {
    RCLCPP_INFO(this->get_logger(), "Socket closed");
  }
#endif

#ifdef UDP
  int sent;
  for (int i = 0; i < 10; i++) {
    RCLCPP_INFO(this->get_logger(), "Sending shutdown message");
    custom_ser::ser_msg("SHUTDOWN", 13, 404, &builder, (uint8_t *)buffer,
                        &size);
    sent = sendto(sockfd, buffer, size, 0, (struct sockaddr *)&dest_addr,
                  sizeof(struct sockaddr_in));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  close(sockfd);
#endif

#ifdef RAW

  char frame[1524];
  custom_ser::ser_msg("SHUTDOWN", 0, 0, &builder, (uint8_t *)&buffer, &size);

  struct ethhdr *eth_write = (struct ethhdr *)frame;
  memcpy(eth_write->h_dest, MAC_122, 6);
  memcpy(eth_write->h_source, MAC_131, 6);
  eth_write->h_proto = htons(CUSTOM_ETHERTYPE);

  size_t frame_len, sent;

  // Add payload after ethernet header
  memcpy(frame + sizeof(struct ethhdr), buffer, size);

  frame_len = sizeof(struct ethhdr) + size;
  // Ensure minimum frame size (64 bytes total)
  // Minimum size for ethernet frames is 64 bytes
  if (frame_len < 64) {
    memset(frame + sizeof(struct ethhdr) + size, 0, 64 - frame_len);
    frame_len = 64;
  }

  // Write the data to the socket
  for (int i = 0; i < 10; i++) {
    sent = sendto(sockfd, frame, frame_len, 0, (struct sockaddr *)&sll,
                  sizeof(sll));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

#endif
  RCLCPP_INFO(this->get_logger(), "Processing data...");
  process_data();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(this->get_logger(), "Shutting down node");
  rclcpp::shutdown();
}

void talker::process_data() {
  // Process the data and write it to the file
  this->file.open(this->logger_name, std::ios::out | std::ios::app);
  if (!this->file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Error occurred during file writing!");
    return;
  }

  int size = 1;

  for (int i = 0; i < msg_size * NB_MSGS; i++) {

    if ((i + 1) % 50 == 0) {
      size *= 10;
    }

    // Check if the message did a round trip
    if (std::get<0>(socket_send_receive_time[i]) == 0 &&
        std::get<1>(socket_send_receive_time[i]) == 0) {
      RCLCPP_WARN(this->get_logger(), "Message %d not received or sent", i);
      lost_packes_counter++;
      continue;
    }

    // Check if the message ID matches
    if (std::get<2>(socket_send_receive_time[i]) !=
        std::get<3>(socket_send_receive_time[i])) {
      RCLCPP_WARN(this->get_logger(),
                  "Message ID mismatch: sent %d, received %d",
                  std::get<2>(socket_send_receive_time[i]),
                  std::get<3>(socket_send_receive_time[i]));
      mistach_counter++;
      continue;
    } else {
      RCLCPP_INFO(this->get_logger(), "Message ID match: sent %d, received %d",
                  std::get<2>(socket_send_receive_time[i]),
                  std::get<3>(socket_send_receive_time[i]));
    }

    // Calculate the elapsed time
    double elapsed_time =
        std::get<1>(socket_send_receive_time[i]) -
        std::get<0>(socket_send_receive_time[i]); // Elapsed time in ms

    if (elapsed_time < 0) {
      continue;
    }

    // Write the data to the file
    this->file << size << " | " << i << " | " << elapsed_time << "\n";
  }

  this->file.close();
  RCLCPP_INFO(this->get_logger(), "Number of mistaches | lost packets: %d",
              mistach_counter);
  RCLCPP_INFO(this->get_logger(), "Number of lost packets: %d",
              lost_packes_counter);
}

bool talker::socket_setup() {

#ifdef TCP
  // STOCK_STREAM means that we are using TCP
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  // Create the socket and bind it to the server port and ip
  if (!socket_tcp::setup_server_socket(&server_sockfd, &serv_addr, port,
                                       &cli_addr, &clilen)) {
    RCLCPP_INFO(this->get_logger(), "Socket setup failed");
    return 0;
  }
  // Check if the sockets were created successfully
  if (!socket_tcp::sync_check(server_sockfd)) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: sync check failed");
    return 0;
  }
#endif

#ifdef UDP
  // SOCK_DGRAM means that we are using UDP
  int targetfd = socket(AF_INET, SOCK_DGRAM, 0); // Create a socket
  sockfd = 17;
  if (dup2(targetfd, sockfd) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR during socket dumping!");
    close(targetfd);
    return -1;
  }

  RCLCPP_INFO(this->get_logger(), "Device UDP socketfd: %d", sockfd);
  if (!socket_udp::sync_check(sockfd, &serv_addr, &dest_addr, port)) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: sync check failed");
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

  // Bind the socket to the WIFI card interface
  if (setup_raw_socket(&sockfd, &sll, MAC_122) == 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: socket setup failed");
    return 0;
  }

  // Perform sync check
  raw_sync_check(sockfd, sll, MAC_122, MAC_131);
  RCLCPP_INFO(this->get_logger(), "Sync done");

#endif

  return 1;
}
