#ifndef COMP_H
#define COMP_H

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

#include "../../latency_test_talker/include/holo_socket.h"

// Serialization includes
#include "../../custom_msg/flatbuff/custom_ser.h"
#include "../../custom_msg/flatbuff/message_generated.h"
#include "flatbuffers/flatbuffers.h"
using TestProtocol::message;

#define PORT 5000
#define SYNC_BUFFER_SIZE 256
#define MAX_FAIL_COUNT 100

#define READY 0
#define START 1
#define STOP 2
#define END 3

/*
holo1: white  | 192.168.0.131
holo2: black  | 192.168.0.122
holo3: red    | 192.168.0.108
holo4: yellow | 192.168.0.109

comp: 192.168.0.72
*/

class comp : public rclcpp::Node {

public:
  comp();

private:
  ////// Variables //////
  /*
  comp_fsm_state:
  0 : check if system in ready
  1 : start procedure
  2 : stop exp
  */
  int comp_fsm_state = READY;
  bool holo_ready[4] = {0}, holo_stop[4] = {0};

  // Sockets //
  struct sockaddr_in sock_addr, broadcast_addr;
  socklen_t socklen;
  int compfd;

  // Flatbuffer //
  std::string msg;
  uint8_t id;
  float value;
  int failure_counter = 0;
  flatbuffers::FlatBufferBuilder builder{1024};
  uint32_t size;

  // Threads //
  bool running = true;
  bool write_enable;

  ////// Function declarations //////
  bool device_UDP_socket(int *sockfd, struct sockaddr_in *sock_addr,
                         struct sockaddr_in *broad_addr);

  void enable_socket_write();
  void exp_start();
};

#endif