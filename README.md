# tello_driver for ROS2 - With Object Detection 


### drone_socket_library:
This is just a library for communication. Some nodes in tello_driver package use this library.

### tf_object_detection: 
ROS2 package for Object Detection using tensorflow. 
References : https://www.codeproject.com/Articles/1268309/Adding-Object-Detection-with-TensorFlow-to-a-Robot
			
### tello_driver: 
ROS2 package for drive tello drone. There are 4 nodes.
-		command_node: For sending command to tello drone
-		state_node: For tello drone current state
-		video_stream_node: For raw video stream from tello drone camera
-		autonomous_flight_node: For aoutonomous drive for tello using 3 nodes above, but not finished.

###  USAGE:
##### 1- ros2 run tello_driver tello_command_node
##### 2- ros2 run tello_driver tello_state_node (optional)
##### 3- ros2 run tello_driver tello_video_stream_node
##### 4-  ros2 run tf_object_detection tf_object_detection_node (This will take time)
##### 5- ros2 run tello_driver autonomous_flight_node (Don't run if tf_object_detection_node is not ready 

