cmake_minimum_required(VERSION 3.5)
project(workstation)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(sim src/sim.cpp )
ament_target_dependencies(sim tf2_ros rclcpp sensor_msgs std_msgs control_input)

add_executable(real_world src/real_world.cpp )
ament_target_dependencies(real_world tf2_ros rclcpp sensor_msgs std_msgs control_input)

ament_python_install_package(${PROJECT_NAME})

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp sensor_msgs std_msgs control_input Eigen3 tf2_ros)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
#target_link_libraries(controller Eigen3::Eigen)

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

install(TARGETS
	controller
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
