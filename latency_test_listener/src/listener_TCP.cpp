#include "listener_TCP.h"

socket_tcp::socket_tcp() {}

bool socket_tcp::setup_client_socket(int sockfd, struct sockaddr_in *serv_addr,
                                     int port, char server_ip[]) {

  struct hostent *server;
  // Get server info
  server = gethostbyname(server_ip);
  if (server == NULL) {
    return 0;
  }

  // Set IPv4
  bzero((char *)serv_addr, sizeof(*serv_addr));
  serv_addr->sin_family = AF_INET;
  // Set server addr in client
  bcopy((char *)server->h_addr, (char *)&serv_addr->sin_addr.s_addr,
        server->h_length);
  // Set server port in client
  serv_addr->sin_port = htons(port);

  // Attempt connection to server
  if (connect(sockfd, (struct sockaddr *)serv_addr, sizeof(*serv_addr)) < 0) {
    return 0;
  }

  int n;
  char buffer[256];

  // Write an ACK from client to server
  strncpy(buffer, "CLIENT_ACK", sizeof(buffer));
  n = write(sockfd, buffer, strlen(buffer));
  if (n < 0) {
    return 0;
  }

  // Wait for response
  bzero(buffer, 256);
  n = read(sockfd, buffer, 255);
  if (n < 0) {
    return 0;
  }

  // Check for server ACK
  if (strncmp(buffer, "SERVER_ACK", sizeof(buffer)) != 0) {
    return 0;
  }

  return 1;
}