#include "talker.h"

// #define TCP
#define UDP

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

  RCLCPP_INFO(this->get_logger(), "Let's rumble!");

  // This thread is called perdiodically to enable writing to the socket every
  // spacing_ms
  std::thread timer_thread(std::bind(&talker::enable_socket_write, this));

  // Start experiment thread
  std::thread exp_thread(std::bind(&talker::socket_exp_launch, this));

  exp_thread.join();
  timer_thread.join();

  // Finish the experiment
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

  // This while loop take ~60µs to execute
  while (running) {
    int ret = poll(&pfd, 1, 0);
    // 1) Check it there is any data that needs to be read on the server socket
    if (ret > 0) {
      if (pfd.revents & POLLIN) {

        // Read the data from the socket
        bzero(buffer, SOCKET_BUFFER_SIZE);
        int n = read(server_sockfd, buffer, SOCKET_BUFFER_SIZE - 1);
        // If there was an issue when reading the socket, skip the iteration
        if (n < 0) {
          break;
        }

        // Set timestamp
        double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;

        // Verify message is valid
        std::string message_id = socket_tcp::extract_message(buffer);
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
      }
    }
    // 2) Send the data to the server
    if (write_enable && ret == 0) {
      // If package size is reached, increase the size
      if ((socket_msg_count + 1) % NB_MSGS == 0) {
        socket_msg_size++;
      }

      // If grace period
      if (grace == true) {
        // Send grace message and count the number of messages sent
        socket_tcp::grace_writer(server_sockfd, &grace_counter_write,
                                 grace_counter_read, &grace);
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
        break;
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

#ifdef UDP

  // Create a poll to monitor the socket
  struct pollfd pfd;
  pfd.fd = sockfd;
  pfd.events = POLLIN; // Monitor for incoming data

  char buffer[SOCKET_BUFFER_SIZE];

  // This while loop take ~60µs to execute
  while (running) {
    int ret = poll(&pfd, 1, 0);
    // 1) Check it there is any data that needs to be read on the server socket
    if (ret > 0) {
      if (pfd.revents & POLLIN) {
        // Read the data from the socket
        bzero(buffer, SOCKET_BUFFER_SIZE);
        int n = recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)&serv_addr,
                         &clilen);

        if (n < 0) {
          continue;
        }

        // Set timestamp
        double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;

        // Verify message is valid
        std::string message_id = socket_tcp::extract_message(buffer);
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
      }
    }
    // 2) Send the data to the server
    if (write_enable && ret == 0) {
      if ((socket_msg_count + 1) % NB_MSGS == 0) {
        socket_msg_size++;
      }

      // If grace period
      if (grace == true) {
        if (grace_counter_write > GRACE_COUNTER_MAX &&
            grace_counter_read > GRACE_COUNTER_MAX) {
          grace = false;
          RCLCPP_INFO(this->get_logger(),
                      "Grace period ended, starting experiment");
          continue;
        }
        std::string msg = "S_131313_";
        strncpy(buffer, msg.c_str(), sizeof(buffer));
        grace_counter_write++;
        int n = sendto(sockfd, buffer, sizeof(buffer), 0,
                       (struct sockaddr *)&dest_addr,
                       sizeof(struct sockaddr_in)); // Send only the msg part of
                                                    // the buffer, until the \0
        if (n < 0) {
          continue;
        }
        write_enable = false;
        continue;
      }

      // If there is no data to be read nor to write, terminate session
      if (socket_msg_count > total_nb_msgs) {
        RCLCPP_INFO(this->get_logger(), "Socket session terminated");
        running = false;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        break;
      }

      // If not grace period
      // Write the data to the socket
      bzero(buffer, SOCKET_BUFFER_SIZE);
      std::string test_string(sizes[socket_msg_size], 'A');

      std::string msg =
          "S_" + std::to_string(socket_msg_count) + "_" + test_string;
      int msg_len = msg.size();
      strncpy(buffer, msg.c_str(), sizeof(buffer));

      int n = sendto(sockfd, buffer, msg_len, 0, (struct sockaddr *)&dest_addr,
                     sizeof(struct sockaddr_in)); // Send only the msg part of
                                                  // the buffer, until the \0
      if (n < 0) {
        continue;
      }

      double sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;
      // Add the time to the socket_send_receive_time array
      std::get<0>(socket_send_receive_time[socket_msg_count]) = sending_time;
      // Add msg id to the socket_send_receive_time array
      std::get<2>(socket_send_receive_time[socket_msg_count]) =
          socket_msg_count;

      socket_msg_count++;

      write_enable = false; // Disable writing until next timer event
    }

    std::this_thread::sleep_for(std::chrono::microseconds(10));
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
  std::string terminate_str = "SHUTDOWN";
  strncpy(buffer, terminate_str.c_str(), sizeof(buffer));
  int n = write(server_sockfd, buffer, strlen(buffer));

  // Close the socket
  close(server_sockfd);
#endif

#ifdef UDP
  for (int i = 0; i < 10; i++) {
    std::string msg = "S_SHUTDOWN_";
    strncpy(buffer, msg.c_str(), sizeof(buffer));
    int n = sendto(sockfd, buffer, sizeof(buffer), 0,
                   (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  close(sockfd);
#endif

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
  sockfd = socket(AF_INET, SOCK_DGRAM, 0); // Create a socket
  if (!socket_udp::sync_check(sockfd, &serv_addr, &dest_addr, port)) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: sync check failed");
    return 0;
  }
#endif

  return 1;
}
