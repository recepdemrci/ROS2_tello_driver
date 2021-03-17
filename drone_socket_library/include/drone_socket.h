#ifndef DRONE_SOCKET_H
#define DRONE_SOCKET_H

#include <string.h>
#include <netinet/in.h>
#include "rclcpp/rclcpp.hpp"

class DroneSocket {
private:
    int sockfd;
    struct sockaddr_in destAddr;
    socklen_t destAddrSize;
public:
    DroneSocket();
    bool init(std::string ipAddr, int port);
    bool bind_();
    void send(std::string command);
    int recv(char *data, int messageSize);
    bool recv(int messageSize);
    bool close_(); 
};

#endif