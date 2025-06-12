#include "talker_UDP.h"

socket_udp::socket_udp() {}

bool socket_udp::sync_check(int sockfd, struct sockaddr_in *serv_addr,
                            struct sockaddr_in *cli_addr, int port) {
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

  socklen_t clilen = sizeof(*cli_addr);
  // Reset all the serv_addr values to zero
  bzero((char *)serv_addr, sizeof(*serv_addr)); // The function bzero() sets all
                                                // values in a buffer to zero

  // Set the serv_addr parameters
  serv_addr->sin_family = AF_INET; // Means that we are using IPv4
  // Tell the socket to accept any of the host machines IPs
  serv_addr->sin_addr.s_addr = INADDR_ANY; // Tells the kernel to bind the
                                           // socket to all available interfaces
  serv_addr->sin_port =
      htons(port); // This sets the port number the server will listen on,
                   // converting it from host byte order to network byte order

  // Bind the socket to the server port and ip
  if (bind(sockfd, (struct sockaddr *)serv_addr, sizeof(*serv_addr)) < 0) {
    return 0;
  }

  int buf_size = 1 << 20; // 1MB
  // setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size));
  setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size));

  // recvfrom: clilen needs a vlaue of type socklen_t, NOT a ptr!
  int sync_msg =
      recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)cli_addr, &clilen);

  if (sync_msg < 0) {
    return 0;
  }

  buffer[sync_msg] = '\0';

  // Send a server sync to the client
  // sendto: clilen needs to be a pointer to a socklen_t
  sync_msg =
      sendto(sockfd, "SERVER_ACK", 10, 0, (struct sockaddr *)cli_addr, clilen);

  // Wait for sync message from the client
  sync_msg =
      recvfrom(sockfd, buffer, 255, 0, (struct sockaddr *)cli_addr, &clilen);

  if (sync_msg < 0) {
    return 0;
  }
  buffer[sync_msg] = '\0';

  if (strncmp(buffer, "CLIENT_ACK", 10) != 0) {
    return 0;
  }

  return 1;
}