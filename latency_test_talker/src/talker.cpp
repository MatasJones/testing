#include "talker.h"

#define SOCKET_MODE

talker::talker() : Node("talker") {

  RCLCPP_INFO(this->get_logger(), "Creating talker");

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

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

  RCLCPP_INFO(this->get_logger(), "Msg size %d ms", msg_size);

  // Setup the logger for later use
  talker::create_logger();

  // This block creates a subscriber on a specified topic and binds it to a
  // callback
  this->talker_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_listener/PI_TO_COMP"), custom_qos,
          std::bind(&talker::get_response_time, this, std::placeholders::_1));

  this->sync_subscriber_ = this->create_subscription<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_IN"), 10,
      std::bind(&talker::echo_sync, this, std::placeholders::_1));

  // Create a publisher on a specified topic
  this->talker_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("COMP_TO_PI"),
                                                            custom_qos);

  this->sync_publisher_ = this->create_publisher<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_OUT"), 10);

  // Set up the experiment parameters from the config file
  talker::setup_experiment();

#ifdef SOCKET_MODE
  // Create a socket and bind it to the server port and ip
  RCLCPP_INFO(this->get_logger(), "Creating socket...");
  if (!talker::socket_setup()) {
    RCLCPP_INFO(this->get_logger(), "Socket setup failed");
    rclcpp::shutdown();

  } else {
    RCLCPP_INFO(this->get_logger(), "Socket setup done");
  }

  RCLCPP_INFO(this->get_logger(), "Let's rumble!");

  std::thread timer_thread(std::bind(&talker::enable_socket_write, this));

  // Start experiment thread
  std::thread exp_thread(std::bind(&talker::socket_exp_launch, this));

  exp_thread.join();
  timer_thread.join();

#endif

#ifdef ROS_MODE
  // Perform synchronization with the listener
  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(SYNC_CHECK_PERIOD),
                              std::bind(&talker::perform_sync, this));

#endif
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

  // Create a poll to monitor the socket
  struct pollfd pfd;
  pfd.fd = server_sockfd;
  pfd.events = POLLIN; // Monitor for incoming data

  while (running) {
    int ret = poll(&pfd, 1, 0);
    // 1) Check it there is any data that needs to be read on the server socket
    if (ret > 0) {
      if (pfd.revents & POLLIN) {
        // Read the data from the socket
        char buffer[256];
        bzero(buffer, 256);
        int n = read(server_sockfd, buffer, 255);
        if (n < 0) {
          RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
          break;
        }
        double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;
        RCLCPP_INFO(this->get_logger(), "Message from client: %s", buffer);

        // Extract msg number from the message
        std::string str_buffer(buffer);
        size_t first = str_buffer.find('_');
        size_t second = str_buffer.find('_', first + 1);
        std::string extracted =
            str_buffer.substr(first + 1, second - first - 1);
        RCLCPP_INFO(this->get_logger(), "Extracted msg number: %s",
                    extracted.c_str());

        // Add the time to the socket_send_receive_time array
        std::get<1>(socket_send_receive_time[atoi(extracted.c_str())]) =
            recieving_time;
      }
    }
    // 2) Send the data to the server
    if (write_enable && ret == 0) {
      // Write the data to the socket
      char buffer[256];
      bzero(buffer, 256);
      std::string msg = "S_" + std::to_string(socket_msg_count) + "_";
      strncpy(buffer, msg.c_str(), sizeof(buffer));

      int n = write(server_sockfd, buffer, strlen(buffer));
      if (n < 0) {
        RCLCPP_ERROR(this->get_logger(), "ERROR writing to socket");
        break;
      }

      double sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;
      std::get<0>(socket_send_receive_time[socket_msg_count]) = sending_time;
      socket_msg_count++;
      write_enable = false; // Disable writing until next timer event
    }
    // If there is no data to be read nor to write, terminate session
    if (socket_msg_count > total_nb_msgs - 1) {
      RCLCPP_INFO(this->get_logger(), "Socket session terminated");
      running = false;
      break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // Close the socket
  close(server_sockfd);
  talker::terminate_exp();
}

void talker::perform_sync() {
  // Create a SyncMsg message
  auto message = custom_msg::msg::SyncMsg();
  bool device_status = true;
  // If the index of the sync array is 0, it means that the listener is not
  // synced yet
  if (sync_array[0] == 0) {
    message.message = "SYNC_CHECK";
    sync_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Sync check sent to listener %d", 1);
    device_status = false;
  }

  // Check if the device is responding
  if (device_status == true) {
    // Check if the talker and listener topic are responding
    if (sync_check == true) {
      this->timer_->cancel();
      RCLCPP_INFO(this->get_logger(),
                  "Sync check done, starting grace period..");
      this->exp_timer_ =
          this->create_wall_timer(std::chrono::milliseconds(spacing_ms_),
                                  std::bind(&talker::run_experiment, this));

      return;
    }
    // Send a message to the listener to check if it is responding on the topic
    auto message = custom_msg::msg::CustomString();
    message.size = 404;
    talker_publisher_->publish(message);
  }

  RCLCPP_INFO(this->get_logger(), "Sync status: holo1: %d", sync_array[0]);

  return;
}

void talker::echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Sync check from listener");
  // If the listener is synced, set the index of the sync array to 1
  sync_array[0] = 1;
}

void talker::get_response_time(
    const custom_msg::msg::CustomString::SharedPtr msg) {

  // Get the current time in microseconds
  double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;

  int index = 0;
  switch (msg->size) {
  case 1:
    break;
  case 10:
    index = 1;
    break;
  case 100:
    index = 2;
    break;
  case 404:
    if (++check_count > NB_SYNC_CHECKS) {
      sync_check = true;
    }
    break;
  case 1000:
    index = 3;
    break;
  case 10000:
    index = 4;
    break;
  case 100000:
    index = 5;
    break;
  case 1000000:
    index = 6;
    break;
  default:
    RCLCPP_INFO(this->get_logger(), "Size not found");
    break;
  }
  std::get<1>(send_receive_time[index][msg->msg_nb]) = recieving_time;
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

void talker::run_experiment() {
  // If the number of iterations for this size is greater than the number of
  // repetitions, increase the size
  if (current_iteration == repetitions) {
    current_iteration = 0;
    current_size++;
  }

  // If the experiment is completed, shutdown the node
  if (static_cast<std::vector<int>::size_type>(current_size) == sizes.size()) {
    if (terminate_set) {
      double current_time = this->get_clock()->now().nanoseconds() / 1.0e6;
      if (current_time - msgs_all_sent_time >= 3000.0) {
        this->exp_timer_->cancel();
        talker::terminate_exp();
      }
      return;
    }
    terminate_set = true;
    msgs_all_sent_time = this->get_clock()->now().nanoseconds() / 1.0e6;
    RCLCPP_INFO(this->get_logger(), "All messages sent to holos");
    return;
  }

  // Prepare the message to be sent
  std::string test_string(sizes[current_size], 'A');
  auto message = custom_msg::msg::CustomString();

  message.msg_nb = current_iteration;
  message.size = sizes[current_size];
  message.message = test_string;

  // Set the send time and publish the message
  double sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;
  talker_publisher_->publish(message);
  std::get<0>(send_receive_time[current_size][current_iteration]) =
      sending_time;

  // RCLCPP_INFO(this->get_logger(), "Message %d of size %d sent",
  //             current_iteration, sizes[current_size]);

  current_iteration++;
}

void talker::terminate_exp() {
  RCLCPP_INFO(this->get_logger(), "Experiment completed");
  // Tell listener nodes to shutdown
  auto message = custom_msg::msg::SyncMsg();
  message.message = "SHUTDOWN";
  sync_publisher_->publish(message);
  process_data();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
}

void talker::process_data() {
  // Process the data and write it to the file
  this->file.open(this->logger_name, std::ios::out | std::ios::app);
  if (!this->file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Error occurred during file writing!");
    return;
  }
#ifdef ROS_MODE
  int size = 1;
  for (int i = 0; i < NB_OF_SIZES; i++) {
    for (int j = 0; j < NB_MSGS; j++) {
      if (std::get<0>(send_receive_time[i][j]) == 0 &&
          std::get<1>(send_receive_time[i][j]) == 0) {
        continue;
      }
      double elapsed_time =
          std::get<1>(send_receive_time[i][j]) -
          std::get<0>(send_receive_time[i][j]); // Elapsed time in ms

      if (elapsed_time < 0) {
        continue;
      }
      this->file << size << " | " << j << " | " << elapsed_time << "\n";
    }
    size *= 10;
  }
#endif

#ifdef SOCKET_MODE

  int size = 1;
  int msg_nb = 0;
  for (int i = 0; i < TOTAL_MSGS; i++) {
    if (std::get<0>(socket_send_receive_time[i]) == 0 &&
        std::get<1>(socket_send_receive_time[i]) == 0) {
      continue;
    }
    double elapsed_time =
        std::get<1>(socket_send_receive_time[i]) -
        std::get<0>(socket_send_receive_time[i]); // Elapsed time in ms

    if (elapsed_time < 0) {
      continue;
    }

    msg_nb++;

    this->file << size << " | " << msg_nb << " | " << elapsed_time << "\n";
    if (i % 49 == 0) {
      size *= 10;
      msg_nb = 0;
    }
  }

#endif

  this->file.close();
}

bool talker::socket_setup() { // Return 1 if socket communication was correctly
                              // setup
  // Socket setup
  int sockfd;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr; // This creates a socket address
  int n;

  /// Create a socket and bind it to the server port and ip
  sockfd = socket(AF_INET, SOCK_STREAM, 0); // Create a socket

  // Verify that a socket was created
  if (sockfd < 0)
    RCLCPP_ERROR(this->get_logger(), "ERROR opening socket");

  // Reset all the serv_addr values to zero
  bzero((char *)&serv_addr, sizeof(serv_addr)); // The function bzero() sets all
                                                // values in a buffer to zero

  // Set the serv_addr parameters
  serv_addr.sin_family = AF_INET;         // Means that we are using IPv4
  serv_addr.sin_addr.s_addr = INADDR_ANY; // Tells the kernel to bind the
                                          // socket to all available interfaces
  serv_addr.sin_port =
      htons(port); // This sets the port number the server will listen on,
                   // converting it from host byte order to network byte order

  // Bind the socket to the server port and ip
  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR on binding");
    return 0;
  }

  // Listen for incoming connections
  listen(sockfd, 5); // arg1: socket file descriptor, arg2: max number of
                     // pending connections

  clilen = sizeof(cli_addr);

  /*
  The accept system call is blocking until a connection is made. It creates a
  new socket file descriptor which should be used for all futher communiqué
  */

  /// Accept a connection from a client and create a new socket file descriptor
  server_sockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

  if (server_sockfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR on accept");
    return 0;
  }

  bzero(buffer, 256);
  /// Read the message from the client
  /*
  Note that the read function is blocking until a message is received on the
  socket (until the client write a msg)
  */
  n = read(server_sockfd, buffer, 255); // n is the number of bytes read

  if (n < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR reading from socket");
    return 0;
  }

  // If there is a pb with the socket, kill the process
  if (strcmp(buffer, "CLIENT_ACK") != 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR, from client ACK");
    return 0;
  }
  RCLCPP_INFO(this->get_logger(), "Message from client: %s", buffer);

  /// Write a response to the client
  n = write(server_sockfd, "SERVER_ACK", 10);

  if (n < 0) {
    RCLCPP_ERROR(this->get_logger(), "ERROR writing to socket");
    return 0;
  }

  // Close the socket
  close(sockfd);
  return 1;
}