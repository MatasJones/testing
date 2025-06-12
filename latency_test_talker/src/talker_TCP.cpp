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

  *clilen = sizeof(*cli_addr);

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

/**
 * @brief This function extracts the message id from the buffer
 *
 * @param buffer
 * @return std::string
 */
std::string socket_tcp::extract_message(char buffer[BUFFER_SIZE]) {
  std::string str_buffer(buffer);
  size_t first = str_buffer.find('_');
  size_t second = str_buffer.find('_', first + 1);
  // Verify that the msg id is extractable
  if (!(first != std::string::npos && second != std::string::npos &&
        second > first)) {
    return "";
  }

  std::string extracted = str_buffer.substr(first + 1, second - first - 1);
  return extracted;
}

/**
 * @brief This function sends a grace message to the client and counts the
 * number of grace messages sent and received back, if that number is greater
 * than MAX, grace period is over
 *
 * @param sockfd is the socket file descriptor
 * @param grace_counter_write is the number of grace messages sent
 * @param grace_counter_read is the number of grace messages received
 * @param grace_status is the status of the grace period
 * @return true if the grace message was sent successfully
 * @return false otherwise
 */
bool socket_tcp::grace_writer(int sockfd, int *grace_counter_write,
                              int grace_counter_read, bool *grace_status,
                              bool custom_ser) {

  if (*grace_counter_write > GRACE_COUNTER_MAX &&
      grace_counter_read > GRACE_COUNTER_MAX) {
    *grace_status = false;
  }

  (*grace_counter_write)++;

  // Perform manual ser grace message
  if (custom_ser == false) {
    std::string msg = "S_131313_";
    int n = write(sockfd, msg.c_str(), msg.size());
    if (n < 0) {
      return 0;
    }
    return 1;
  }

  /*
  Not the most efficient way of doing this, but it works and is only used for
  setup
  */

  // Perform flatbuffer grace message
  flatbuffers::FlatBufferBuilder builder{1024};
  char buf[1024];
  uint32_t size;
  custom_ser::ser_msg("GRACE", *grace_counter_write, 404, &builder,
                      (uint8_t *)buf, &size);

  int n = write(sockfd, buf, size);
  if (n < 0) {
    return 0;
  }

  return 1;
}
