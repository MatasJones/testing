#include "talker.h"

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
  this->talker_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_listener/PI_TO_COMP"), custom_qos,
          std::bind(&talker::get_response_time, this, std::placeholders::_1));

  this->sync_subscriber_ = this->create_subscription<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_IN"), QUEUE_SIZE,
      std::bind(&talker::echo_sync, this, std::placeholders::_1));

  // Create a publisher on a specified topic
  this->talker_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("COMP_TO_PI"),
                                                            custom_qos);

  this->sync_publisher_ = this->create_publisher<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_OUT"), QUEUE_SIZE);

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
      this->grace_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(GRACE_PERIOD),
          std::bind(&talker::terminate_grace_period, this));
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

void talker::terminate_grace_period() {
  RCLCPP_INFO(this->get_logger(), "Grace period over");
  RCLCPP_INFO(this->get_logger(), "Starting experiment!");
  this->grace_timer_->cancel();
  this->exp_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(spacing_ms_),
                              std::bind(&talker::run_experiment, this));
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
  this->file.close();
}