#ifndef HOLO_H
#define HOLO_H

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

#include "talker_TCP.h"
#include "talker_UDP.h"

// Serialization includes
#include "../../custom_msg/flatbuff/message_generated.h"
#include "flatbuffers/flatbuffers.h"
using TestProtocol::message;

/*
holo1: white  | 192.168.0.131
holo2: black  | 192.168.0.122
holo3: red    | 192.168.0.108
holo4: yellow | 192.168.0.109
*/

class holo : public rclcpp::Node {

public:
  holo();

private:
  //
  int b;
  //
};

#endif