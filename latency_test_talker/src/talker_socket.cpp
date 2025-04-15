/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

void error(const char *msg) {
  perror(msg);
  exit(1);
}

int main(int argc, char *argv[]) {

  /// Initialize the variables
  int sockfd, newsockfd, portno; // Initialize the file descriptors

  socklen_t clilen;

  char buffer[256];

  struct sockaddr_in serv_addr, cli_addr; // This creates a socket address

  int n;

  if (argc < 2) { // Check if the port number is provided
    fprintf(stderr, "ERROR, no port provided\n");
    exit(1);
  }
  printf("Starting server on port %s\n", argv[1]);
  /// Create a socket and bind it to the server port and ip
  sockfd = socket(AF_INET, SOCK_STREAM, 0); // Create a socket

  if (sockfd < 0)
    error("ERROR opening socket");

  bzero((char *)&serv_addr, sizeof(serv_addr)); // The function bzero() sets all
                                                // values in a buffer to zero
  portno = atoi(argv[1]); // atoi() function to convert this from a string of
                          // digits to an integer, needed to convert the given
                          // port nb to int

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);

  if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");

  /// Listen for incoming connections

  listen(sockfd, 5); // arg1: socket file descriptor, arg2: max number of
                     // pending connections

  clilen = sizeof(cli_addr);

  /*
  The accept system call is blocking until a connection is made. It creates a
  new socket file descriptor which should be used for all futher communiqué
  */
  /// Accept a connection from a client and create a new socket file descriptor
  newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

  if (newsockfd < 0)
    error("ERROR on accept");

  bzero(buffer, 256);
  /// Read the message from the client
  /*
  Note that the read function is blocking until a message is received on the
  socket (until the client write a msg)
  */
  n = read(newsockfd, buffer, 255);

  if (n < 0)
    error("ERROR reading from socket");

  printf("Here is the message: %s\n", buffer);
  /// Write a response to the client
  n = write(newsockfd, "I got your message", 18);

  if (n < 0)
    error("ERROR writing to socket");

  close(newsockfd);
  close(sockfd);
  return 0;
}

/*
 typedef struct sockaddr_in {
#if ...
  short          sin_family;
#else
  ADDRESS_FAMILY sin_family;
#endif
  USHORT         sin_port;
  IN_ADDR        sin_addr;
  CHAR           sin_zero[8];
} SOCKADDR_IN, *PSOCKADDR_IN;
 */