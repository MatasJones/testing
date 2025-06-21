#include "listener_UDP.h"

socket_udp::socket_udp() {}

bool socket_udp::sync_check(int sockfd, struct sockaddr_in *serv_addr,
                            struct sockaddr_in *cli_addr, int port) {

  char buffer[256];

  socklen_t clilen = sizeof(*cli_addr);

  // Set the serv_addr parameters
  bzero((char *)serv_addr, sizeof(serv_addr));
  serv_addr->sin_family = AF_INET; // Means that we are using IPv4
  serv_addr->sin_addr.s_addr = inet_addr("192.168.131"); // Set the server IP
  serv_addr->sin_port = htons(5000);

  // Setup cli_addr then bind the socket
  bzero((char *)cli_addr, sizeof(cli_addr));
  cli_addr->sin_family = AF_INET;
  cli_addr->sin_addr.s_addr = INADDR_ANY;
  cli_addr->sin_port = htons(port);

  if (bind(sockfd, (struct sockaddr *)cli_addr, sizeof(*cli_addr)) < 0) {
    return 0;
  }

  int sync_msg =
      sendto(sockfd, "First msg", 9, 0, (struct sockaddr *)serv_addr, clilen);

  buffer[sync_msg] = '\0';

  if (sync_msg < 0) {
    return 0;
  }

  // Wait for a response from the server
  sync_msg =
      recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)cli_addr, &clilen);

  if (strncmp(buffer, "SERVER_ACK", 10) != 0) {
    return 0;
  }
  // Send acknowledgment to the server
  sync_msg =
      sendto(sockfd, "CLIENT_ACK", 10, 0, (struct sockaddr *)cli_addr, clilen);

  return 1;
}