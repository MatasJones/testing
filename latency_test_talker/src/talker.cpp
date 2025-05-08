#include "talker.h"

#define OG_QOS_MODE
// #define CUSTOM_QOS_MODE
#define LOG_MODE

talker::talker() : Node("talker") {

  RCLCPP_INFO(this->get_logger(), "Creating talker");

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(QUEUE_SIZE))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  this->declare_parameter("spacing_ms",
                          DEAFULT_SPACING); // Default value of 400ms
  spacing_ms_ = this->get_parameter("spacing_ms")
                    .as_int(); // Set the spacing as the value passed or as the
                               // default value
  RCLCPP_INFO(this->get_logger(), "Spacing time %d ms", spacing_ms_);
  talker::create_logger();

  RCLCPP_INFO(this->get_logger(), "cwd %s", cwd.c_str());
// This block creates a subscriber on a specified topic and binds it to a
// callback
#ifdef CUSTOM_QOS_MODE
  this->talker_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_listener/PI_TO_COMP"), custom_qos,
          std::bind(&talker::get_response_time, this, std::placeholders::_1));

  // Create a publisher on a specified topic
  this->talker_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("COMP_TO_PI"),
                                                            custom_qos);
#endif

#ifdef OG_QOS_MODE
  this->talker_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_listener/PI_TO_COMP"), 10,
          std::bind(&talker::get_response_time, this, std::placeholders::_1));

  this->talker_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("COMP_TO_PI"), 10);

#endif

  this->sync_subscriber_ = this->create_subscription<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_IN"), 10,
      std::bind(&talker::echo_sync, this, std::placeholders::_1));

  this->sync_publisher_ = this->create_publisher<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_OUT"), 10);

  // Set up the experiment parameters from the config file
  talker::setup_experiment();

  // Perform synchronization with the listener
  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(SYNC_CHECK_PERIOD),
                              std::bind(&talker::perform_sync, this));

  RCLCPP_INFO(this->get_logger(), "Talker created");
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
      // Wait for 5 seconds before starting the experiment
      std::this_thread::sleep_for(std::chrono::seconds(5));
      this->exp_timer_ =
          this->create_wall_timer(std::chrono::milliseconds(spacing_ms_),
                                  std::bind(&talker::run_experiment, this));

      return;
    }
    // Send a message to the listener to check if it is responding on the topic
    auto message = custom_msg::msg::CustomString();
    message.size = 404;
    message.msg_id = sync_counter++;
    message.message = "PING";
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
  std::get<1>(send_receive_time[msg->msg_id]) = recieving_time;
  std::get<3>(send_receive_time[msg->msg_id]) = msg->msg_id;
#ifdef LOG_MODE
  RCLCPP_INFO(this->get_logger(), "Msg %d of size %d received", msg->msg_id,
              msg->size);
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

void talker::test_timer() {
  test_timer_->cancel();
  test_timer_ = nullptr;
  test_continue = true;
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
      if (current_time - msgs_all_sent_time >= 5000.0) {
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
  // std::string test_string(sizes[1], 'A'); //////// Set back to OG after
  // testing
  std::string test_string(sizes[current_size], 'A');
  auto message = custom_msg::msg::CustomString();

  message.msg_id = msg_id;
  message.size = sizes[current_size];
  message.message = test_string;

  // Set the send time and publish the message
  double sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;
  talker_publisher_->publish(message);

  std::get<0>(send_receive_time[msg_id]) = sending_time;
  std::get<2>(send_receive_time[msg_id]) = msg_id;

#ifdef LOG_MODE
  RCLCPP_INFO(this->get_logger(), "Message %d of size %d sent", msg_id,
              sizes[current_size]);
#endif
  current_iteration++;
  msg_id++;
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
  int size = 1;
  for (int i = 0; i < NB_OF_SIZES * NB_MSGS; i++) {

    if (i % NB_MSGS == 0 && i != 0) {
      size *= 10;
    }

    if (std::get<0>(send_receive_time[i]) == 0 ||
        std::get<1>(send_receive_time[i]) == 0) {
      RCLCPP_WARN(this->get_logger(), "Message %d not received or sent", i);
      lost_packes_counter++;
      continue;
    }
    if (std::get<2>(send_receive_time[i]) !=
        std::get<3>(send_receive_time[i])) {
      RCLCPP_WARN(
          this->get_logger(), "Message ID mismatch: sent %d, received %d",
          std::get<2>(send_receive_time[i]), std::get<3>(send_receive_time[i]));
      mistach_counter++;
      continue;
    } else {
      RCLCPP_INFO(this->get_logger(), "Message ID match: sent %d, received %d",
                  std::get<2>(send_receive_time[i]),
                  std::get<3>(send_receive_time[i]));
    }
    double elapsed_time =
        std::get<1>(send_receive_time[i]) -
        std::get<0>(send_receive_time[i]); // Elapsed time in ms

    if (elapsed_time < 0) {
      continue;
    }
    this->file << size << " | " << i << " | " << elapsed_time << "\n";
  }

  this->file.close();
  RCLCPP_INFO(this->get_logger(), "Number of mistaches: %d", mistach_counter);
  RCLCPP_INFO(this->get_logger(), "Number of lost packets: %d",
              lost_packes_counter);
}