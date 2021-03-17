#include <iostream>
#include <string.h> 
#include <unistd.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#include "drone_socket.h"

#define TIMEOUT_SEC 5

using namespace std;


DroneSocket::DroneSocket() {}

// Create connection and initialize it based on ip address and port number
bool DroneSocket::init(string ipAddr, int port){  
    this->sockfd = socket(AF_INET, SOCK_DGRAM, 0);                                      // Create socket
    if (this->sockfd < 0){                                                              // Control if any failure occured
        return false;
    }
    memset(&this->destAddr, 0, sizeof(this->destAddr));                                 // Allocate memory for addres            
    this->destAddr.sin_family = AF_INET;                                                 
    this->destAddr.sin_port = htons(port);                                              // Assign destination port 
    if (inet_pton (AF_INET, ipAddr.c_str(), &this->destAddr.sin_addr) == -1) {          // Assign destination ip adress
        return false;
    }
    this->destAddrSize = sizeof(this->destAddr);                                        // Set destination address size
    return true;
}

// NOTE: bind function can be use just for server socket. Because client doen't have to start communication
bool DroneSocket::bind_(){
    if (bind(this->sockfd, (struct sockaddr *) &this->destAddr, this->destAddrSize) < 0){
	    return false;
    }
    return true;
}

// Sends data to destination address
void DroneSocket::send(string command){
    sendto(this->sockfd, command.c_str(), command.size(), MSG_CONFIRM, (const struct sockaddr *) &this->destAddr, this->destAddrSize);
}

// Receives data from desstinaion address
// If any error occured, returns negative value
int DroneSocket::recv(char *data, int messageSize){
    int n, rv;
    fd_set set;
    struct timeval timeout;
    
    FD_ZERO(&set);                                                                      // Clear the set
    FD_SET(this->sockfd, &set);                                                         // Add our file descriptor to the set
    timeout.tv_sec = TIMEOUT_SEC;                                                       // Set time out
    timeout.tv_usec = 0;
    rv = select(this->sockfd + 1, &set, NULL, NULL, &timeout);
    if (rv > 0) {                                                                       // Check if there is anything to receive in socket
        n = recvfrom(this->sockfd, data, messageSize, MSG_WAITALL, (struct sockaddr *) &this->destAddr, &this->destAddrSize);
        if (n > 0){
            data[n] = '\0';
        }
        return n;                                                                       // If receive is successful return data size
    }
    else {
        return rv;                                                                      // If anty error occured, return negative value as error
    }
}

// Receives data from desstinaion address
// And returns true if ok, returns false if error
bool DroneSocket::recv(int messageSize){
    int n, rv;
    fd_set set;
    struct timeval timeout;
    std::string ok = "ok";
    char *data = (char *)malloc(messageSize);
    
    FD_ZERO(&set);                                                                      // Clear the set
    FD_SET(this->sockfd, &set);                                                         // Add our file descriptor to the set
    timeout.tv_sec = TIMEOUT_SEC;                                                       // Set time out
    timeout.tv_usec = 0;
    rv = select(this->sockfd + 1, &set, NULL, NULL, &timeout);
    if (rv <= 0) {
        return false;
    }
    n = recvfrom(this->sockfd, data, messageSize, MSG_WAITALL, (struct sockaddr *) &this->destAddr, &this->destAddrSize);
    if (n <= 0) {
        return false;
    }
    data[n] = '\0';
    if (ok.compare(data) == 0) {
        return true;
    }
    return false;
}

// Close socket when you finish your job
bool DroneSocket::close_(){
    if(close(this->sockfd) < 0) {
        return false;
    }
    return true;
}