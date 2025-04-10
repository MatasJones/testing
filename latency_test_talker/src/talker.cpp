#include "talker.h"

talker::talker() : Node("talker"), count_(0) {

  RCLCPP_INFO(this->get_logger(), "Creating talker");
  talker::create_logger();

  RCLCPP_INFO(this->get_logger(), "cwd %s", cwd.c_str());
  // This block creates a subscriber on a specified topic and binds it to a
  // callback
  this->talker_subscriber_ =
      this->create_subscription<custom_msg::msg::CustomString>(
          ("/latency_test_listener/PI_TO_COMP"), 10,
          std::bind(&talker::get_response_time, this, std::placeholders::_1));

  this->sync_subscriber_ = this->create_subscription<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_IN"), 10,
      std::bind(&talker::echo_sync, this, std::placeholders::_1));

  // Create a publisher on a specified topic
  this->talker_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("COMP_TO_PI"), 10);

  this->sync_publisher_ = this->create_publisher<custom_msg::msg::SyncMsg>(
      ("/latency_test_talker/SYNC_TOPIC_OUT"), 10);

  this->declare_parameter("spacing_ms", 400);
  spacing_ms_ = this->get_parameter("spacing_ms").as_int();

  // Set up the experiment parameters from the config file
  talker::setup_experiment();

  // Run the experiment
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&talker::perform_sync, this));

  RCLCPP_INFO(this->get_logger(), "Talker created");
}

void talker::echo_sync(const custom_msg::msg::SyncMsg::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Sync check from listener");
  // If the listener is synced, set the index of the sync array to 1
  sync_array[0] = 1;
}

void talker::perform_sync() {
  // Create a SyncMsg message
  auto message = custom_msg::msg::SyncMsg();
  bool sync_status = true;
  // If the index of the sync array is 0, it means that the listener is not
  // synced yet
  if (sync_array[0] == 0) {
    message.message = "SYNC_CHECK";
    sync_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Sync check sent to listener %d", 1);
    sync_status = false;
  }

  if (sync_status == true) {
    RCLCPP_INFO(this->get_logger(), "Sync check done!");
    this->timer_->cancel();
    this->exp_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(spacing_ms_),
                                std::bind(&talker::run_experiment, this));
  }

  RCLCPP_INFO(this->get_logger(), "Sync status: holo1: %d", sync_array[0]);

  return;
}

void talker::get_response_time(
    const custom_msg::msg::CustomString::SharedPtr msg) {

  // Get the current time in microseconds
  double recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;
  // Extract the sending time from the map
  auto it = latency_map.find(std::make_tuple(msg->msg_nb, msg->size));
  // Substract the recieve and send timestamps to get elapsed time in
  // microseconds
  double elapsed_time =
      recieving_time -
      std::get<2>(it->second); // it->second gets the value not the key

  std::string log = std::to_string(msg->size) + " | " +
                    std::to_string(msg->msg_nb) + " | " +
                    std::to_string(elapsed_time);

  talker::log(log);
  // Log the elapsed time in microseconds
  RCLCPP_INFO(this->get_logger(), "Msg %d of size %d received", msg->msg_nb,
              msg->size);
  RCLCPP_INFO(this->get_logger(), "Elapsed time: %f ms", elapsed_time);
}

void talker::create_logger() {

  this->file.open(this->logger_name, std::ios::out | std::ios::trunc);

  if (!this->file.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Error occurred during file creation!");
    return;
  }

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
  RCLCPP_INFO(this->get_logger(), "Setting up experiment..");

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
    RCLCPP_INFO(this->get_logger(), "Curently size %d", sizes[current_size]);
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

  // Map the message msg nb, size and send time to the latency map
  talker::map_time(message.msg_nb, sizes[current_size], sending_time);

  RCLCPP_INFO(this->get_logger(), "Message %d of size %d sent",
              current_iteration, sizes[current_size]);

  current_iteration++;
}

void talker::map_time(int msg_nb, int size, double time) {
  latency_map.insert(std::make_pair(std::make_tuple(msg_nb, size),
                                    std::make_tuple(msg_nb, size, time)));
}

void talker::terminate_exp() {
  RCLCPP_INFO(this->get_logger(), "Experiment completed");
  // Tell listener nodes to shutdown
  auto message = custom_msg::msg::SyncMsg();
  message.message = "SHUTDOWN";
  sync_publisher_->publish(message);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
}