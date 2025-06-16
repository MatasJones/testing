#ifndef HOLO_H
#define HOLO_H

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/qos.hpp"
#include <chrono>
#include <cmath>
#include <deque>
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
#include <net/if.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <poll.h>

#include "holo_socket.h"
#include "talker_TCP.h"
#include "talker_UDP.h"

// Serialization includes
#include "../../custom_msg/flatbuff/message_generated.h"
#include "flatbuffers/flatbuffers.h"
using TestProtocol::message;

#define PORT 5002
#define COMP_PORT 5000
#define SYNC_BUFFER_SIZE 256
#define MAX_FAIL_COUNT 100

/*
holo1: white  | 192.168.0.131
holo2: black  | 192.168.0.122
holo3: red    | 192.168.0.108
holo4: yellow | 192.168.0.109

comp: 192.168.0.72
*/

struct ip_addrs {
  std::string device_ip;
  int nb_neigh;
  std::string neigh_ip[2];
};

struct sync_valid {
  bool device;
  bool neigh;
};

class holo : public rclcpp::Node {

public:
  holo();

private:
  ////// Variables //////
  int neigh_1_last_ip_digit, neigh_2_last_ip_digit;
  //
  struct ip_addrs ip_addr;

  // Sockets //
  struct sockaddr_in sock_addr[2], dest_addr[2], comp_addr;
  int sockfd[2];
  socklen_t socklen[2] = {sizeof(sock_addr[0]), sizeof(sock_addr[1])};
  struct sync_valid sync_validated[2];
  bool write_enable[2] = {0};
  bool averaging = false;

  // Flatbuffer //
  std::string msg;
  uint8_t id;
  float value;
  int failure_counter = 0;
  flatbuffers::FlatBufferBuilder builder{1024};
  uint32_t size;

  // Threads //
  bool sync_check = true, running = true;
  bool sync_success = false;

  ////// Function declarations //////
  void get_ip(struct ip_addrs *this_ip_addrs);
  bool device_UDP_socket(struct ip_addrs this_ip_addrs, int sockfd[],
                         struct sockaddr_in sock_addr[],
                         struct sockaddr_in dest_addr[]);
  void holo_holo_sync();
  void enable_socket_write();
  void perform_exp();
  void avg_enable();
};

#endif