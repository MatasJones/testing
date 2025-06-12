#ifndef HOLO_SOCKET_H
#define HOLO_SOCKET_H

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

bool setup_UDP_socket(int sockfd, struct sockaddr_in *sock_addr, int port);

#endif