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

add_executable(driver src/driver.cpp)
ament_target_dependencies(driver rclcpp sensor_msgs std_msgs) # File not found error

add_executable(transform_broadcaster src/transform_broadcaster.cpp)
ament_target_dependencies(transform_broadcaster tf2_ros rclcpp sensor_msgs std_msgs)

add_executable(sim src/sim.cpp )
ament_target_dependencies(sim tf2_ros rclcpp sensor_msgs std_msgs control_input)

add_executable(real_world src/real_world.cpp )
ament_target_dependencies(real_world tf2_ros rclcpp sensor_msgs std_msgs control_input)

ament_python_install_package(${PROJECT_NAME})

add_executable(motor_state src/motor_state.cpp)
ament_target_dependencies(motor_state rclcpp sensor_msgs std_msgs dynamixel_sdk control_input)

install(TARGETS
    driver
    DESTINATION lib/${PROJECT_NAME}
    )

install(TARGETS
    transform_broadcaster
    DESTINATION lib/${PROJECT_NAME}
    )
    
install(TARGETS
    sim
    DESTINATION lib/${PROJECT_NAME}
    )

install(TARGETS
    real_world
    DESTINATION lib/${PROJECT_NAME}
    )
install(PROGRAMS
    scripts/gui.py
    DESTINATION lib/${PROJECT_NAME}
    )

install (TARGETS
motor_state
    DESTINATION lib/${PROJECT_NAME}
    )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
    DIRECTORY config description launch
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()
