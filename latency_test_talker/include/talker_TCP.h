
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

class socket_tcp {

public:
  socket_tcp();

  static bool setup_server_socket(int *sockfd, struct sockaddr_in *serv_addr,
                                  int port, struct sockaddr_in *cli_addr,
                                  socklen_t *clilen);

  static bool sync_check(int sockfd);

private:
};

#endif