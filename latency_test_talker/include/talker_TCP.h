
#ifndef TALKER_TCP_H
#define TALKER_TCP_H

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
#include <string>
#include <tuple>

#include "../../custom_msg/flatbuff/custom_ser.h"

#define BUFFER_SIZE 65490
#define GRACE_COUNTER_MAX 20

class socket_tcp {

public:
  socket_tcp();

  static bool setup_server_socket(int *sockfd, struct sockaddr_in *serv_addr,
                                  int port, struct sockaddr_in *cli_addr,
                                  socklen_t *clilen);

  static bool sync_check(int sockfd);

  static std::string extract_message(char buffer[BUFFER_SIZE]);

  static bool grace_writer(int sockfd, int *grace_counter_write,
                           int grace_counter_read, bool *grace_status,
                           bool custom_ser);

private:
};

#endif