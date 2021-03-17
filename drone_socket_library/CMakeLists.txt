cmake_minimum_required(VERSION 3.5)
project(drone_socket_library)

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

# ----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
# ----------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/drone_socket.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
# ----------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/
  DESTINATION include)
# ---------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
