#include "talker_UDP.h"

socket_udp::socket_udp() {}

bool socket_udp::sync_check(int sockfd, struct sockaddr_in *dest_addr,
                            socklen_t *clilen) {
  // UDP is sessionless, so we don't need to accept a connection
  // This means that for every message received, there is also the clients
  // socket info
  /*
  Which consists a struct: struct sockaddr_in cli_addr
  1) cli_add.sin_family = AF_INET (IPv4)
  2) cli_addr.sin_port : the client's port number
  3) cli_addr.sin_addr : the client's IP address
  */
  char buffer[256];
  bzero(buffer, 256);

  struct sockaddr_in cli_addr;

  int sync_msg =
      recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)&cli_addr, clilen);

  if (sync_msg < 0) {
    return 0;
  }

  buffer[sync_msg] = '\0';
  // Copy the client addr to the dest addr for future usage
  memcpy(dest_addr, &cli_addr, sizeof(struct sockaddr_in));
  *clilen = sizeof(cli_addr);

  // Send a server sync to the client
  sync_msg = sendto(sockfd, "SERVER_ACK", 10, 0, (struct sockaddr *)dest_addr,
                    sizeof(struct sockaddr_in));

  // Wait for sync message from the client
  sync_msg =
      recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)&cli_addr, clilen);

  if (sync_msg < 0) {
    return 0;
  }
  buffer[sync_msg] = '\0';

  if (strncmp(buffer, "CLIENT_ACK", 10) != 0) {
    return 0;
  }

  return 1;
}