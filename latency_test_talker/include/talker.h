#ifndef TALKER_H
#define TALKER_H

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/qos.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <time.h>
#include <tuple>
#include <vector>
#include <yaml-cpp/yaml.h>

// Socket includes
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <poll.h>

#include "talker_RAW.h"
#include "talker_TCP.h"
#include "talker_UDP.h"

// Serialization includes
// #include "../../custom_msg/flatbuff/custom_ser.h" // defined in talker_tcp.h
#include "../../custom_msg/flatbuff/message_generated.h"
#include "flatbuffers/flatbuffers.h"
using TestProtocol::message;

#define NB_LISTENERS 1
#define QUEUE_SIZE 100
#define NB_SYNC_CHECKS 10
#define DEAFULT_SPACING 100
#define SYNC_CHECK_PERIOD 200

#define NB_MSGS 50
#define NB_OF_SIZES 7
#define TOTAL_MSGS (NB_MSGS * NB_OF_SIZES)
#define DEFAULT_MSG_SIZE 3
#define SOCKET_BUFFER_SIZE 65490
#define GRACE_COUNTER_MAX 20

#define MAX_FAIL_COUNT 100

using namespace std;

class talker : public rclcpp::Node {
public:
  talker();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr exp_timer_;
  rclcpp::TimerBase::SharedPtr socket_timer_;

  int spacing_ms_;
  int msg_size;

  double msgs_all_sent_time;
  std::ofstream file;
  std::string logger_name = "/home/testing/dev_ws/src/testing_logs/logger.csv";

  // Experiment parameters
  int repetitions;
  std::vector<int> sizes;
  std::vector<int> ids;
  int current_iteration = 0;
  int current_size = 0;
  int sync_array[NB_LISTENERS] = {0};
  bool terminate_set = false;
  bool sync_check = false;
  int check_count = 0;
  bool running = true;

  int port = 5000;
  int server_sockfd;
  bool write_enable = true;
  int total_nb_msgs;
  bool grace = true;
  int grace_counter_write = 0;
  int grace_counter_read = 0;

  // Experiment variables
  int socket_msg_count = 0;
  int socket_msg_size = 0;

  // Logging variables
  int lost_packes_counter = 0;
  int mistach_counter = 0;

  int sockfd;
  struct sockaddr_in dest_addr;
  socklen_t clilen = sizeof(cli_addr);
  struct sockaddr_in serv_addr, cli_addr; // This creates a socket address

  // RAW socket
  struct sockaddr_ll sll;

  // Flatbuffer variables
  flatbuffers::FlatBufferBuilder builder{1024};
  uint8_t *buf;
  uint32_t size;
  bool custom_seri = false;
  std::string msg;
  uint8_t id;
  int32_t value;
  int failure_counter = 0;

  std::tuple<double, double, uint16_t, uint16_t>
      socket_send_receive_time[TOTAL_MSGS] = {};

  std::filesystem::path cwd = std::filesystem::current_path();

  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  void create_logger();
  void setup_experiment();
  void terminate_exp();
  void process_data();
  bool socket_setup();
  void enable_socket_write();
  void socket_exp_launch();

  template <typename T> void log(T data);
};

#endif