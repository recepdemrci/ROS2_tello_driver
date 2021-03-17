#!/usr/bin/env python
from __future__ import print_function # TODO is this needed
import sys
import rclpy
from rclpy.node import Node
from tf_object_detection.object_detection_lib import ObjectDetection
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('tf_object_detection')
        self.__image_pub = self.create_publisher(
            Image, "tf_object_detection/result_image", 1)                       # Publisher to publish update image
        self.__result_pub = self.create_publisher(
            String, "tf_object_detection/result", 1)                            # Publisher to publish the result
        self.__image_sub = self.create_subscription(
            Image, "tello/camera", self.image_callback, 10)                     # Subscriber to supply the image fom the camera
        self.declare_parameter("live_stream", False)                            # Declare parmeter for show or publish result image 
        self.declare_parameter("path", "/tf/models/research/object_detection")  # Write the path for models/research/object_detection directory
        self.declare_parameter("confidence_level", 0.6)                         # wrtie the confidence level, any object with a level below this will not be used
        
        self.__bridge = CvBridge()
        self.get_logger().info("LOADING object detection model (This may take long)")
        confidence_level = self.get_parameter(
            "confidence_level").get_parameter_value().double_value         
        object_detection_path = self.get_parameter(
            "path").get_parameter_value().string_value                     
        self.__odc = ObjectDetection(object_detection_path, confidence_level)   # Create the object_detection_lib class instance        
            
    # Callback for new image received
    def image_callback(self, data):
        image = self.__bridge.imgmsg_to_cv2(data, "bgr8")                       # Convert the ROS image to an OpenCV image
        object_names_detected = self.__odc.scan_for_objects(image)              # The supplied image will be modified if known objects are detected
        
        if self.get_parameter("live_stream").get_parameter_value().bool_value:   
            cv2.imshow("Object Detection Result", image)
            cv2.waitKey(1)
        else:
            try:
                self.__image_pub.publish(self.__bridge.cv2_to_imgmsg(image, "bgr8"))   # publish the image, it may have been modified
            except CvBridgeError as e:
                print(e)

            result = String()
            result.data = object_names_detected
            self.__result_pub.publish(result)                                   # Publish names of objects detected


def main(args=None):
    rclpy.init(args=args)  
    odn = ObjectDetectionNode()
    odn.get_logger().info("READY")
    try:
        rclpy.spin(odn)
    except KeyboardInterrupt:
        print("SHUTTING DOWN")

if __name__ == '__main__':
    main()
