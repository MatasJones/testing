#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

void error(const char *msg) {
  perror(msg);
  exit(0);
}

int main(int argc, char *argv[]) {
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;

  char buffer[256];

  portno = atoi(argv[2]);
  // Create a socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");

  // Get the server address and search for it
  server = gethostbyname(argv[1]);
  if (server == NULL) {
    fprintf(stderr, "ERROR, no such host\n");
    exit(0);
  }
  /// Initialize the server address
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(portno);

  /// Attempt connection to server
  if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR connecting");
  printf("Please enter the message: ");

  // Generate the data to be sent
  bzero(buffer, 256);
  fgets(buffer, 255, stdin);
  n = write(sockfd, buffer, strlen(buffer));
  if (n < 0)
    error("ERROR writing to socket");

  // Read the response from the server
  bzero(buffer, 256);
  n = read(sockfd, buffer, 255);
  if (n < 0)
    error("ERROR reading from socket");
  printf("%s\n", buffer);

  // Close the socket
  close(sockfd);
  return 0;
}

/**
struct  hostent
{
    char    *h_name;        //// official name of host /
    char **h_aliases;             / alias list /
    int h_addrtype;               / host address type /
    int h_length;                 / length of address /
    char **h_addr_list;           / list of addresses from name server /
    #define h_addr h_addr_list[0] / address, for backward compatiblity /
};
*/