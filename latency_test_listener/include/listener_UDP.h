
#ifndef LISTENER_UDP_H
#define LISTENER_UDP_H

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

#define BUFFER_SIZE 65490
#define GRACE_COUNTER_MAX 20

class socket_udp {

public:
  socket_udp();

  static bool sync_check(int sockfd, struct sockaddr_in *serv_addr,
                         struct sockaddr_in *cli_addr, int port);

private:
};

#endif