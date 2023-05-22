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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(driver src/driver.cpp)
ament_target_dependencies(driver rclcpp sensor_msgs std_msgs) # File not found error

add_executable(transform_broadcaster src/transform_broadcaster.cpp)
ament_target_dependencies(transform_broadcaster tf2_ros rclcpp sensor_msgs std_msgs)

add_executable(motor_state src/motor_state.cpp)
ament_target_dependencies(motor_state rclcpp sensor_msgs std_msgs dynamixel_sdk)

# Made for Dynamixel libraries
#include_directories(include/dynamixel_sdk)

#set(DYNAMIXEL_SDK_SOURCES
	#src/dynamixel_sdk/packet_handler.cpp
	#src/dynamixel_sdk/protocol1_packet_handler.cpp
	#src/dynamixel_sdk/protocol2_packet_handler.cpp
	#src/dynamixel_sdk/group_sync_read.cpp
	#src/dynamixel_sdk/group_sync_write.cpp
	#src/dynamixel_sdk/group_bulk_read.cpp
	#src/dynamixel_sdk/group_bulk_write.cpp
	#src/dynamixel_sdk/port_handler.cpp
	#src/dynamixel_sdk/port_handler_arduino.cpp
	#include/dynamixel_sdk/dynamixel_sdk.h
	#include/dynamixel_sdk/port_handler.h
	#include/dynamixel_sdk/group_bulk_read.h
	#include/dynamixel_sdk/group_bulk_write.h
	#include/dynamixel_sdk/group_sync_read.h
	#include/dynamixel_sdk/group_sync_write.h
	#include/dynamixel_sdk/packet_handler.h
	#include/dynamixel_sdk/port_handler.h
	#include/dynamixel_sdk/protocol1_packet_handler.h
	#include/dynamixel_sdk/protocol2_packet_handler.h
	#include/dynamixel_sdk/port_handler_arduino.h
#)

#if(APPLE)
  #add_library(dynamixel_sdk SHARED
    #${DYNAMIXEL_SDK_SOURCES}
    #src/dynamixel_sdk/port_handler_mac.cpp
    #include/dynamixel_sdk/port_handler_mac.h
  #)
#elseif(WIN32)
  #add_library(dynamixel_sdk SHARED
    #${DYNAMIXEL_SDK_SOURCES}
    #src/dynamixel_sdk/port_handler_windows.cpp
    #include/dynamixel_sdk/port_handler_windows.h
  #)
#else()
  #add_library(dynamixel SHARED
    #${DYNAMIXEL_SDK_SOURCES}
    #src/dynamixel_sdk/port_handler_linux.cpp
    #include/dynamixel_sdk/port_handler_linux.h
  #)
#endif()

#install(
  #DIRECTORY include/
  #DESTINATION include/
#)

#install(
  #TARGETS ${PROJECT_NAME}
  #ARCHIVE DESTINATION lib
  #LIBRARY DESTINATION lib
  #RUNTIME DESTINATION bin
#)

#install(TARGETS
	#LIBRARY ${DYNAMIXEL_SDK_SOURCES}
    #DESTINATION lib/${PROJECT_NAME}/dynamixel_sdk
    #)
#


install(TARGETS
    driver
    DESTINATION lib/${PROJECT_NAME}
    )

install(TARGETS
    transform_broadcaster
    DESTINATION lib/${PROJECT_NAME}
    )

install(TARGETS
    motor_state
    DESTINATION lib/${PROJECT_NAME}
    )

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

install(
    DIRECTORY config description launch
    DESTINATION share/${PROJECT_NAME}
    )

#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
ament_package()
