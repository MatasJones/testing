#include "talker_TCP.h"

socket_tcp::socket_tcp() {}

/**
 * @brief This function sets up the server socket for TCP communication
 *
 * Warning: this function only allows for the connection to one client at a time
 *
 * @param sockfd is the socket file descriptor
 * @param serv_addr is the server address structure
 * @return true if socket creation successful
 * @return false otherwise
 */
bool socket_tcp::setup_server_socket(int *sockfd, struct sockaddr_in *serv_addr,
                                     int port, struct sockaddr_in *cli_addr,
                                     socklen_t *clilen) {

  // Setup sockfd
  *sockfd = socket(AF_INET, SOCK_STREAM, 0);
  // Check if an error has occurred during sockfd init
  if (*sockfd < 0) {
    return 0;
  }

  // Reset all the serv_addr values to zero
  bzero((char *)serv_addr, sizeof(serv_addr));

  // Set the serv_addr parameters
  serv_addr->sin_family = AF_INET; // Means that we are using IPv4
  // Tell the socket to accept any of the host machines IPs
  serv_addr->sin_addr.s_addr = INADDR_ANY; // Tells the kernel to bind the
                                           // socket to all available interfaces
  serv_addr->sin_port =
      htons(port); // This sets the port number the server will listen on,
                   // converting it from host byte order to network byte order

  // Bind the socket to the server port and ip
  if (bind(*sockfd, (struct sockaddr *)serv_addr, sizeof(*serv_addr)) < 0) {
    return 0;
  }

  // Listen for incoming connections
  listen(*sockfd, 5);

  /*
  The accept system call is blocking until a connection is made. It
  creates a new socket file descriptor which should be used for all
  futher communiqué
  */

  /// Accept a connection from a client and create a new socket file
  /// descriptor
  *sockfd = accept(*sockfd, (struct sockaddr *)cli_addr, clilen);

  if (*sockfd < 0) {
    return 0;
  }

  return 1;
}

/**
 * @brief This function performs a synchronization check with the client
 *
 * @param sockfd is the socket file descriptor
 * @return true if the sync check is successful
 * @return false otherwise
 */
bool socket_tcp::sync_check(int sockfd) {
  char buffer[256];
  // Read the message from the client
  int n = read(sockfd, buffer, 255);
  if (n < 0) {
    return 0;
  }
  // Check for client ACK
  if (strcmp(buffer, "CLIENT_ACK") != 0) {
    return 0;
  }
  // Send ACK to the client
  n = write(sockfd, "SERVER_ACK", 10);
  if (n < 0) {
    return 0;
  }
  // Sync check done
  return 1;
}
