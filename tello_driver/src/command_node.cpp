#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_driver/srv/trigger.hpp"
#include "drone_socket.h"

#define TELLO_IP "192.168.10.1"
#define PORT 8889
#define MAX_MSG_SIZE 1024
#define PUB_PERIOD 12000ms
#define linear_const 500
#define angular_const 360
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


// This node for sending command to the drone 
class command_node : public rclcpp::Node {
private:
	DroneSocket socket;
	rclcpp::Service<tello_driver::srv::Trigger>::SharedPtr cmd_srv;			// Service of command(for takeoff, land, streamon etc..)
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sdk;			// Subcriber of command(for Tello SDK command only)
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;		// Subscriber of command(for Twist message)
	rclcpp::TimerBase::SharedPtr timer_;									// Timer for keep connection alive


public:
	// Constructor 
	// Create Subscriber for cmd_sdk: command using Tello SDK
	// Create Subscriver for cmd_vel: vellocity command using geometry_msgs/Twist
	// Initialize socket and get ready for sending command to drone(Control any if error occured)
	command_node () : Node("tello_command") {
		this->cmd_srv = this->create_service<tello_driver::srv::Trigger>(
			"tello/cmd_srv", std::bind(&command_node::cmd_srv_callback, this, _1, _2));
		this->cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
			"tello/cmd_vel", 10, std::bind(&command_node::cmd_vel_callback, this, _1));
		this->cmd_sdk = this->create_subscription<std_msgs::msg::String>(
			"tello/cmd_sdk", 10, std::bind(&command_node::cmd_sdk_callback, this, _1));
		this->timer_ = this->create_wall_timer(
            PUB_PERIOD, std::bind(&command_node::keep_alive, this));

		if (this->init_()) {
			RCLCPP_INFO(this->get_logger(), "READY");
		}
		else {
			rclcpp::shutdown();
		}
	}

	~command_node() {
		RCLCPP_INFO(this->get_logger(), "SHUTTING DOWN...");
		try {
			this->socket.send("land");
			this->socket.close_();
		} catch (...) { }
	}

private:
	// Init socket communication
	bool init_() {
		char *rc_data = (char *)malloc(MAX_MSG_SIZE);

		if ( ! this->socket.init(TELLO_IP, PORT)) {							// Initialize DroneSocket for communication,
			RCLCPP_ERROR(this->get_logger(), "FAILED socket initialize");	// and control if any error occured.
			return false;
		}
		this->socket.send("command");										// Send "command" to drone for first time connection,
		if(this->socket.recv(rc_data, MAX_MSG_SIZE) <= 0) {
			RCLCPP_ERROR(this->get_logger(), "FAILED drone connection");
			return false;
		}
		return true;
	}

	// Keep socket connection alive by sending message to the drone in every 12 secs
	void keep_alive() {
		char *rc_data = (char *)malloc(MAX_MSG_SIZE);

		this->socket.send("time?");
		if (this->socket.recv(rc_data, MAX_MSG_SIZE) <= 0) {
			RCLCPP_ERROR(this->get_logger(), "No Connection"); 
		}
        else {
			this->socket.recv(rc_data, MAX_MSG_SIZE);
			RCLCPP_INFO(this->get_logger(), "Flight Time: %s", rc_data);
		}
	}

	// Callback function for sending command to the drone
	// This works as service-client connection
	void cmd_srv_callback(tello_driver::srv::Trigger::Request::SharedPtr request,
	tello_driver::srv::Trigger::Response::SharedPtr response) {	
		this->socket.send(request->command);
		if (this->socket.recv(MAX_MSG_SIZE)) {
			response->success = true;
			response->message = "ok";
		}
		else {
			response->success = false;
			response->message = "error";
		}
	}

	// Callback function for sending command to the drone (using Tello SDK command)
	// This works as publisher-subscriber connection
	void cmd_sdk_callback(const std_msgs::msg::String::SharedPtr message) {
		char *rc_data = (char *)malloc(MAX_MSG_SIZE);

		this->socket.send(message->data.c_str());
		if (this->socket.recv(rc_data, MAX_MSG_SIZE) <= 0) {
			RCLCPP_ERROR(this->get_logger(), "No Connection");
		}
		else {
			this->socket.recv(rc_data, MAX_MSG_SIZE);
			RCLCPP_INFO(this->get_logger(), "%s", rc_data);
		}
	}

	// Callback function for sending cmd_vel to the drone (using geometry_msgs/Twist)
	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr message) {
		std::string x;														// linear x
		std::string y;														// linear y
		std::string z;														// linear z
		std::string w;														// angular z  
		std::string command;
		
		x = this->format_(message->linear.x, 100, -100);
		y = this->format_(message->linear.y, 100, -100);
		z = this->format_(message->linear.z, 100, -100);
		w = this->format_(message->angular.z, 100, -100);

		command.append("rc ");
		command.append(x).append(" ");
		command.append(y).append(" ");
		command.append(z).append(" ");		
		command.append(w);		
		this->socket.send(command);
	}

	std::string format_(float value, float max, float min) {
		if (value > max) {
			value = max;
		}
		if (value < min) {
			value = min;
		}
		std::string value_s = std::to_string(value);
		return value_s.substr(0, value_s.find(".")+3);
	}
};


// Initialize rclcpp::tello_command and create command_node object
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto drone = std::make_shared<command_node>();
	while (rclcpp::ok()) {
		rclcpp::spin(drone);   
	}
	return 0;
}