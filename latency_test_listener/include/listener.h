#ifndef LISTENER_H
#define LISTENER_H

#include "rclcpp/rclcpp.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <netdb.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "listener_TCP.h"

#define QUEUE_SIZE 100
#define SOCKET_BUFFER_SIZE 65490

class listener : public rclcpp::Node {

public:
  listener();
  int count_ = 0;

private:
  bool socket_setup();

  std::string ip_addr;
  std::filesystem::path cwd = std::filesystem::current_path();
  std::string config_file_path = cwd.string() + "/src/config/config.yaml";

  int id;
  int port = 5000;
  char server_ip[13] = "192.168.0.72"; // Server IP
  int sockfd;
  int power = 0;

  bool running = true;

  struct sockaddr_in serv_addr;
  socklen_t addr_len = sizeof(serv_addr);
};
#endif // LISTENER_H