#include "holo.h"

holo::holo() : Node("holo") {

  RCLCPP_INFO(this->get_logger(), "Starting holo.");

  // Lookup this device's IP address and find neighbouring devices
  RCLCPP_INFO(this->get_logger(), "Looking up device IP and sequence nb...");

  struct ip_addrs ip_addr;
  holo::get_ip(&ip_addr);
  RCLCPP_INFO(
      this->get_logger(),
      "Device IP addr: %s, nb neigh: %d, neigh ip 1: %s, neigh ip 2: %s",
      (ip_addr.device_ip).c_str(), ip_addr.nb_neigh,
      (ip_addr.neigh_ip[0]).c_str(), (ip_addr.neigh_ip[1]).c_str());
}

void holo::get_ip(struct ip_addrs *this_ip_addrs) {

  int n;
  struct ifreq ifr;
  char array[] = "wlan0";
  n = socket(AF_INET, SOCK_DGRAM, 0);
  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, array, IFNAMSIZ - 1);
  ioctl(n, SIOCGIFADDR, &ifr);
  close(n);

  this_ip_addrs->device_ip =
      inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);

  char last = this_ip_addrs->device_ip.back();
  int ip_last_digit = last - '0';

  // Holo order left | 131 | 122 | 108 | 109 | right
  switch (ip_last_digit) {
  case 1:
    // Has only 1 neighbour
    this_ip_addrs->nb_neigh = 1;
    this_ip_addrs->neigh_ip[0] = "192.168.9.122";
    break;
  case 2:
    // Has 2 neighbours
    this_ip_addrs->nb_neigh = 2;
    this_ip_addrs->neigh_ip[0] = "192.168.9.131";
    this_ip_addrs->neigh_ip[1] = "192.168.9.108";
    break;
  case 8:
    // Has 2 neighbours
    this_ip_addrs->nb_neigh = 2;
    this_ip_addrs->neigh_ip[0] = "192.168.9.122";
    this_ip_addrs->neigh_ip[1] = "192.168.9.109";
    break;

  case 9:
    // Has only 1 neighbour
    this_ip_addrs->nb_neigh = 1;
    this_ip_addrs->neigh_ip[0] = "192.168.9.108";
    break;
  }

  return;
}