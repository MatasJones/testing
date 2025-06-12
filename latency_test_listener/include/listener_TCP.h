
#ifndef LISTENER_TCP_H
#define LISTENER_TCP_H

// Socket includes
#include <arpa/inet.h>
#include <netdb.h>
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

class socket_tcp {

public:
  socket_tcp();

  static bool setup_client_socket(int sockfd, struct sockaddr_in *serv_addr,
                                  int port, char server_ip[]);

private:
};

#endif