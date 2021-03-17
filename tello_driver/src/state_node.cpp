#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "drone_socket.h"

#define TELLO_IP "0.0.0.0"
#define PORT 8890
#define MAX_MSG_SIZE 2048
#define PUB_PERIOD 100ms
using namespace std::chrono_literals;

// TODO: Rosbag kullan
// This node for receiving state of the drone 
class state_node : public rclcpp::Node {
private:
    DroneSocket socket;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state;              // Publisher for status of the drone
    rclcpp::TimerBase::SharedPtr timer_;                                    // and timer for publisher

public:
    // Constructor 
    // Create Publisher for state: communicates with drone and publishes received data
    // Start socket connection for receiving state of the drone 
    state_node () : Node("tello_state") {
        this->state = this->create_publisher<std_msgs::msg::String>("tello/state", 1);
        this->timer_ = this->create_wall_timer(
            PUB_PERIOD, std::bind(&state_node::publish_callback, this));

        if (this->init_()) {
            RCLCPP_INFO(this->get_logger(), "READY");
        }
        else {
            rclcpp::shutdown();
        }
    }

    ~state_node() {
        RCLCPP_INFO(this->get_logger(), "SHUTTING DOWN...");
        try {
            this->socket.close_();
        } catch (...) { }
    }

private:
    // init socket communication
    bool init_() {
        if ( ! this->socket.init(TELLO_IP, PORT)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED socket initialize");
            return false;
        }
        if ( ! this->socket.bind_()) {
            RCLCPP_ERROR(this->get_logger(), "FAILED socket bind");
            return false;
        }
        return true;
    }

    // Publishes received state data of the drone (using std_msgs::String message)
    void publish_callback() {
        std_msgs::msg::String message;
        char *rc_data = (char *)malloc(MAX_MSG_SIZE);
    
        if (this->socket.recv(rc_data, MAX_MSG_SIZE) > 0) {                 // Receives state of the drone through socket
            message.data = rc_data;                                         // Create std_msgs::String and publish
            this->state->publish(message);
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "No Connection");
        }
    }
};


// Initialize rclcpp::tello_state and create state_node object
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto drone = std::make_shared<state_node>();
    while (rclcpp::ok()) {
        rclcpp::spin(drone);
    }
    return 0;
}