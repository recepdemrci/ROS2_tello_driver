#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tello_driver/srv/trigger.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define TELLO_IP "0.0.0.0"
#define PORT 11111
#define MAX_MSG_SIZE 65536
#define PUB_PERIOD 0.5ms
using namespace std::chrono_literals;

/* TODO: Rosbag lazım olabilir onu implement et
*/

// This node for live video stream
class video_stream_node : public rclcpp::Node {
private:
    cv::Mat frame;                                                          // Frames of video from drone camera
    cv::VideoCapture video_capture;                                         // VideoCapture for drone camera
    rclcpp::Client<tello_driver::srv::Trigger>::SharedPtr cmd_client;       // Client for start connection by sending streamon command
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_stream;     // Video stream publisher for object detection 
    rclcpp::TimerBase::SharedPtr timer_live;                                // Timer for live video stream
    rclcpp::TimerBase::SharedPtr timer_publish;                             // Timer for publish video stream


public:
    // Constructor
    // Create client for start stream connection
    // Create publisher for publish video stream
    // Declare parameters with default value
    video_stream_node() : Node("tello_video_stream")  {
        this->cmd_client = this->create_client<tello_driver::srv::Trigger>(
            "tello/cmd_srv");
        this->video_stream = this->create_publisher<sensor_msgs::msg::Image>(
            "tello/camera", 1);                                                 // Create publisher, topic:/tello/camera
        this->declare_parameter("live", false);                                 // Decleare parameters with default value
        this->declare_parameter("publish", true);

        if (this->get_parameter("live").as_bool()) {                            // Get live_stream parameter
            this->timer_live = this->create_wall_timer(                         // If it is true, then create timer for live stream
                PUB_PERIOD, std::bind(&video_stream_node::live_stream, this));  
        }
        if (this->get_parameter("publish").as_bool()) {                         // Get publish_stream parameter
            this->timer_publish = this->create_wall_timer(                      // f it is true then create timer for publish stream
                PUB_PERIOD, std::bind(&video_stream_node::publish_stream, this));
        }

        if (this->init_()) {
            this->set_video_capture();        
            RCLCPP_INFO(this->get_logger(), "READY");
        }
        else {
            rclcpp::shutdown();
        }
    }

    ~video_stream_node() {
        RCLCPP_INFO(this->get_logger(), "SHUTTING DOWN...");
        try {
            this->video_capture.release();
            cv::destroyAllWindows();
        } catch (...) { }
    }

private:
    // Send "streamon" command to the tello/cmd_srv for START video stream
    bool init_() {
        std::string streamon = "streamon";
        
        while ( ! this->cmd_client->wait_for_service(1s)) {
            if ( ! rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                "INTERRUPTED while waiting for the service");
                return false;
            }
            RCLCPP_WARN(this->get_logger(), 
            "Waiting for tello_command");
        }
        
        auto request = std::make_shared<tello_driver::srv::Trigger::Request>();
        request->command = streamon;
        auto result = this->cmd_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(),
             "FAILED to receive response from tello_command");
            return false;
        }
        if( ! result.get()->success) {
            RCLCPP_ERROR(this->get_logger(),
             "FAILED to start video_stream");
            return false;
        }
        return true;
    }

    // Live video stream of drone camera
    void live_stream() {    
        this->video_capture.read(this->frame);                                  // Read a frame from drone camera
        if( ! this->frame.empty()) {                                            // If it is not empty, show the frame
            cv::imshow("Tello Live", this->frame);
            cv::waitKey(1);
        }
    }

    // Publish video stream of drone camera
    void publish_stream() {
        sensor_msgs::msg::Image msg_image;                                      // Create sensor_msgs::ImagePtr object for publish video frame

        this->video_capture.read(this->frame);                                  // Read a frame from drone camera
        if( ! this->frame.empty()) {                                            // If it is not empty, publish it
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", this->frame).toImageMsg(msg_image);
            this->video_stream->publish(msg_image);
        }
    }
    
    // Get an address for udp video connection with drone
    // return : "udp://@0.0.0.0:11111"
    std::string get_udp_video_address() {
        std::string result = "udp://@";
        result.append(TELLO_IP);
        result.append(":");
        result.append(std::to_string(PORT));
        return result;
    }

    // Try many times to connect drone camera 
    void set_video_capture() {
        std::string udp_video_address;
        
        udp_video_address = this->get_udp_video_address();                      // Get udp connection address: "udp://@0.0.0.0:11111"
        try {
            this->video_capture = cv::VideoCapture(udp_video_address);          // Try to get video capture, and open it
            while ( ! this->video_capture.isOpened()) {
                this->video_capture.open(udp_video_address);
            } 
        } catch(...) { }
    }
};


int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto drone = std::make_shared<video_stream_node>();
    while(rclcpp::ok()) {
        rclcpp::spin(drone);
    }    
    return 0;
}