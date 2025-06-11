#include "listener_RAW.h"

/**
 * @brief Set the up raw socket object for layer 2 communiqué
 *
 * @param sockfd
 * @param sll A device-independent physical-layer addr, to be completed
 * @param dest MAC addr of the destination
 * @return true if socket correctly established
 * @return false otherwise
 */
bool setup_raw_socket(int *sockfd, struct sockaddr_ll *sll,
                      unsigned char dest[6]) {

  if (*sockfd < 0) {
    close(*sockfd);
    return 0;
  }

  // socket Set sll to 0
  memset(sll, 0, sizeof(struct sockaddr_ll));

  // Set necessary information in sll
  sll->sll_family = AF_PACKET;
  sll->sll_protocol =
      htons(CUSTOM_ETHERTYPE); // Set the protocol to capture all frames
  sll->sll_ifindex = if_nametoindex(INTERFACE_NAME); // Get the interface index
  sll->sll_halen = 6;                                // MAC addr length (6)
  memcpy(sll->sll_addr, dest, ETH_ALEN); // copy your 6-byte MAC into the array

  // Check if interface name is valid
  if (sll->sll_ifindex == 0) {
    close(*sockfd);
    return 0;
  }

  // bind the socket for later reading
  if (bind(*sockfd, (struct sockaddr *)sll, sizeof(struct sockaddr_ll)) == -1) {
    return 0;
  }
  return 1;
}

/**
 * @brief Perform an ACK exchange with the peer
 *
 * @param sockfd
 * @param sll
 * @param dest
 * @param src
 * @return true if the sync check was succesful
 * @return false otherwise
 */
void raw_sync_check(int sockfd, struct sockaddr_ll sll, unsigned char dest[6],
                    unsigned char src[6]) {
  // Await s_ACK
  unsigned char buffer[1514]; // Max Ethernet frame size
  ssize_t bytes_received;
  char msg[6];

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    bytes_received = recv(sockfd, buffer, sizeof(buffer), 0);
    if (bytes_received < 1) {
      continue;
    }
    // Check if frame is large enough to contain Ethernet header
    if (bytes_received < sizeof(struct ethhdr)) {
      continue;
    }

    struct ethhdr *eth_sync = (struct ethhdr *)buffer;
    if (ntohs(eth_sync->h_proto) != CUSTOM_ETHERTYPE) {
      continue; // Skip non-custom frames
    }
    // Correct MAC address comparison using memcmp()
    if (memcmp(eth_sync->h_dest, src, 6) != 0 ||
        memcmp(eth_sync->h_source, dest, 6) != 0) {
      continue; // Skip frames not intended for us
    }

    // Read payload
    unsigned char *payload = buffer + sizeof(struct ethhdr);

    memcpy(msg, payload, 6);
    msg[5] = '\0'; // Safe null-termination
    if (memcmp(msg, "S_ACK", 5) == 0) {
      break;
    }
  }

  // Send C_ACK
  // Create a frame [eth header] + [payload]
  unsigned char frame[1514];
  memset(frame, 0, sizeof(frame)); // Initialize frame buffer

  // Create an ethernet header pointer
  struct ethhdr *eth = (struct ethhdr *)frame;
  memcpy(eth->h_dest, dest, 6);
  memcpy(eth->h_source, src, 6);
  eth->h_proto = htons(CUSTOM_ETHERTYPE);

  // Add payload after ethernet header
  char c_ACK[] = "C_ACK\0";
  size_t msg_len = strlen(c_ACK);
  memcpy(frame + sizeof(struct ethhdr), c_ACK, msg_len);

  size_t frame_len = sizeof(struct ethhdr) + msg_len;
  // Ensure minimum frame size (64 bytes total)
  // Minimum size for ethernet frames is 64 bytes
  if (frame_len < 64) {
    memset(frame + sizeof(struct ethhdr) + msg_len, 0, 64 - frame_len);
    frame_len = 64;
  }
  ssize_t sent =
      sendto(sockfd, frame, frame_len, 0, (struct sockaddr *)&sll, sizeof(sll));
  return;
}