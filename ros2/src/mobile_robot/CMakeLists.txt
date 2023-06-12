cmake_minimum_required(VERSION 3.5)
project(mobile_robot)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_input REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(robot_state src/robot_state.cpp)
ament_target_dependencies(robot_state rclcpp sensor_msgs std_msgs dynamixel_sdk control_input tf2_ros)

install (TARGETS
robot_state
    DESTINATION lib/${PROJECT_NAME}
    )

    if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
    DIRECTORY description launch
    DESTINATION share/${PROJECT_NAME}
    )
    
ament_package()
