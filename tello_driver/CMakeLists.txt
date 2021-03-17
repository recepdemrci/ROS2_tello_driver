cmake_minimum_required(VERSION 3.5)
project(tello_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_socket_library REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------------ for service -------------------------------
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/Trigger.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
 )
ament_export_dependencies(rosidl_default_runtime)
# ----------------------------------------------------------------

include_directories(include)
ament_export_include_directories(include)

add_executable(tello_command src/command_node.cpp)
add_executable(tello_state src/state_node.cpp)
add_executable(tello_video_stream src/video_stream_node.cpp)
add_executable(autonomous_flight src/autonomous_flight_node.cpp)

ament_target_dependencies(tello_command rclcpp std_msgs geometry_msgs drone_socket_library)
ament_target_dependencies(tello_state rclcpp std_msgs drone_socket_library)
ament_target_dependencies(tello_video_stream rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(autonomous_flight rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

install(TARGETS
  tello_command
  tello_state
  tello_video_stream
  autonomous_flight
  DESTINATION lib/${PROJECT_NAME})

# ------------------------------- for service ------------------------------------------
rosidl_target_interfaces(tello_command ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(tello_video_stream ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(autonomous_flight ${PROJECT_NAME} "rosidl_typesupport_cpp")
# --------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
