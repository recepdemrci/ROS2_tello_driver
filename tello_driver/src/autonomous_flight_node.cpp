#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tello_driver/srv/trigger.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;

class autonomous_flight_node : public rclcpp::Node {
private:
    rclcpp::Client<tello_driver::srv::Trigger>::SharedPtr cmd_client;       // Client for start connection by sending streamon command
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_vel;            // ROS Publisher through the /cmd_vel topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tf_od_result;    // Object detection results as string split with '-' charachter 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr tf_od_image;   // Adjusted image after object detection


public:
    autonomous_flight_node() : Node("autonomous_flight") {
        this->cmd_client = this->create_client<tello_driver::srv::Trigger>(
            "tello/cmd_srv");
        this->cmd_vel = this->create_publisher<std_msgs::msg::String>(
            "tello/cmd_vel", 1);
        this->tf_od_result = this->create_subscription<std_msgs::msg::String>(
            "tf_object_detection/result", 10, std::bind(&autonomous_flight_node::tf_od_result_callback, this, _1));
        this->tf_od_image = this->create_subscription<sensor_msgs::msg::Image>(
            "tf_object_detection/result_image", 10, std::bind(&autonomous_flight_node::tf_od_result_image_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "READY");
    }

    ~autonomous_flight_node() {
        RCLCPP_INFO(this->get_logger(), "SHUTTING DOWN...");
        try {
            this->cmd_client_send("land");
            cv::destroyAllWindows();
        } catch (...) { }
    }

private:
    // Send command throug cmd_srv(for takeoff, land etc..) 
    bool cmd_client_send(std::string command) {
        while ( ! this->cmd_client->wait_for_service(4s)) {
            if ( ! rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                "INTERRUPTED while waiting for the service");
                return false;
            }
            RCLCPP_WARN(this->get_logger(), 
            "Waiting for tello_command");
        }

        auto request = std::make_shared<tello_driver::srv::Trigger::Request>();
        request->command = command;
        auto result = this->cmd_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
             "FAILED to receive response from tello_command");
            return false;
        }
        if( ! result.get()->success) {
            RCLCPP_ERROR(this->get_logger(),
             "FAILED %s", command);
            return false;
        }
        return true;
    }

    void tf_od_result_image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
        const cv_bridge::CvImageConstPtr out = cv_bridge::toCvCopy(image_msg, "bgr8");
        cv::imshow("Drone Object Detection", out->image);
        cv::waitKey(1);
	}

    void tf_od_result_callback(const std_msgs::msg::String::SharedPtr message){
       /*  std::string result = message->data.c_str();
        std::size_t found = result.find("person");
        if (found != std::string::npos) {
            this->cmd_client_send("up 35");
            this->cmd_client_send("down 35");
        } */
    }  
};


int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto drone = std::make_shared<autonomous_flight_node>();
    while(rclcpp::ok()) {
        rclcpp::spin(drone);
    }    
    return 0;
}