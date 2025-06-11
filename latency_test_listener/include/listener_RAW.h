#ifndef LISTENER_RAW_H
#define LISTENER_RAW_H

#include <arpa/inet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <netpacket/packet.h>
#include <string.h>
#include <sys/socket.h>
#include <thread>
#include <time.h>
#include <unistd.h>

#define CUSTOM_ETHERTYPE 0x88B5

const char INTERFACE_NAME[] = "wlan0";
// use: ip link show dev <interface>, to get MAC addr
static unsigned char MAC_131[6] = {0x98, 0x03, 0xcf,
                                   0xd2, 0x24, 0x50}; // WHITE (TALKER)
static unsigned char MAC_122[6] = {0x98, 0x03, 0xcf,
                                   0xd2, 0x24, 0x14}; // BLACK (LISTENER)

bool setup_raw_socket(int *sockfd, struct sockaddr_ll *sll,
                      unsigned char dest[6]);

void raw_sync_check(int sockfd, struct sockaddr_ll sll, unsigned char dest[6],
                    unsigned char src[6]);
#endif
