#include "holo_socket.h"

bool setup_UDP_socket(int sockfd, struct sockaddr_in *sock_addr, int port) {
  // Reset all the sock_addr values to zero
  bzero((char *)sock_addr, sizeof(*sock_addr));

  // Set the sock_addr parameters
  sock_addr->sin_family = AF_INET;
  sock_addr->sin_addr.s_addr = INADDR_ANY;
  sock_addr->sin_port = htons(port);

  // Bind the socket to the server port and ip
  if (bind(sockfd, (struct sockaddr *)sock_addr, sizeof(*sock_addr)) < 0) {
    return false;
  }

  return true;
}