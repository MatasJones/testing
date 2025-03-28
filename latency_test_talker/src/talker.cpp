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

  // Create a publisher on a specified topic
  this->talker_publisher_ =
      this->create_publisher<custom_msg::msg::CustomString>(("COMP_TO_PI"), 10);

  // Set up the experiment parameters from the config file
  talker::setup_experiment();

  // Sync timer
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&talker::perform_sync, this));

  // This block creates a publisher on a specified topic and binds it to a
  // timer callback
  // this->timer_ = this->create_wall_timer(
  //     std::chrono::milliseconds(200), std::bind(&talker::run_experiment,
  //     this)); // Uncomment this line when ready
}

void talker::perform_sync() {
  auto message = std_msgs::msg::String();
  message = "AKW";
  sync_publisher_->publish(message);
}

void talker::get_response_time(
    const custom_msg::msg::CustomString::SharedPtr msg) {

  // Get the current time in microseconds
  recieving_time = this->get_clock()->now().nanoseconds() / 1.0e6;

  // substract the recieve and send timestamps to get elapsed time in
  // microseconds
  double elapsed_time = recieving_time - sending_time;
  std::time_t time = std::time(nullptr);

  std::string timestamp = std::string(std::asctime(std::gmtime(&time)));
  timestamp.pop_back(); // Remove trailing '\n'

  std::string log = timestamp + " | " + std::to_string(msg->size) + " | " +
                    std::to_string(elapsed_time);

  talker::log(log);
  // Log the elapsed time in microseconds
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
  }

  // If the experiment is completed, shutdown the node
  if (static_cast<std::vector<int>::size_type>(current_size) == sizes.size()) {
    RCLCPP_INFO(this->get_logger(), "Experiment completed");
    rclcpp::shutdown();
  }

  // Prepare the message to be sent
  std::string test_string(sizes[current_size], 'A');
  auto message = custom_msg::msg::CustomString();
  message.id = current_iteration;
  message.size = sizes[current_size];
  message.message = test_string;

  // Set the send time and publish the message
  sending_time = this->get_clock()->now().nanoseconds() / 1.0e6;
  talker_publisher_->publish(message);

  RCLCPP_INFO(this->get_logger(), "Message %d of size %d sent",
              current_iteration, sizes[current_size]);
  current_iteration++;
}