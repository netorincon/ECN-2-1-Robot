// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetVelocity.h"
#include "dynamixel_sdk_examples/SetVelocity.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk_examples/Position.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_VELOCITY    104
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132
#define ADDR_OPERATING_MODE   11

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define DXL3_ID               3               // DXL3 ID
#define DXL4_ID               4               // DXL4 ID

#define BAUDRATE              1000000         // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyACM0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" or "$ ls /dev/ttyACM*" command
//#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" or "$ ls /dev/ttyACM*" command

PortHandler * portHandler;
PacketHandler * packetHandler;



ros::Publisher get_position_pub;

// Get current velocity with service

bool getPresentVelocityCallback(
  dynamixel_sdk_examples::GetVelocity::Request & req,
  dynamixel_sdk_examples::GetVelocity::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t velocity = 0;

  // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_VELOCITY, (uint32_t *)&velocity, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getVelocity : [ID:%d] -> [VELOCITY:%d]", req.id, velocity);
    res.velocity = velocity;
    return true;
  } else {
    ROS_INFO("Failed to get velocity! Result: %d", dxl_comm_result);
    return false;
  }
}

// Set velocity

void setVelocityCallback(const dynamixel_sdk_examples::SetVelocity::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t velocity = (unsigned int)msg->velocity; // Convert int32 -> uint32
 
  // Write Goal Velocity (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)msg->id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id, msg->velocity);
  } else {
    ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
  } 
}

// Set position

void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t position = (unsigned int)msg->position; // Convert int32 -> uint32
 
  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

// Get position in a topic

void timer_callback(const ros::TimerEvent&)
{  
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  
  int32_t position1 = 0;
  int32_t position2 = 0;
  int32_t position3 = 0;
  int32_t position4 = 0;
  
  dynamixel_sdk_examples::Position pos_motors;
  pos_motors.id = {0, 0, 0, 0};
  pos_motors.position = {0, 0, 0, 0};

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, 1, ADDR_PRESENT_POSITION, (uint32_t *)&position1, &dxl_error);   // portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position1, &dxl_error);   
  if (dxl_comm_result == COMM_SUCCESS) { 
    pos_motors.id[0] = 1;   
    pos_motors.position[0] = position1;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
  }
  
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, 2, ADDR_PRESENT_POSITION, (uint32_t *)&position2, &dxl_error);   // portHandler, DXL2_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position2, &dxl_error);  
  if (dxl_comm_result == COMM_SUCCESS) { 
    pos_motors.id[1] = 2;   
    pos_motors.position[1] = position2;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
  }
  
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, 3, ADDR_PRESENT_POSITION, (uint32_t *)&position3, &dxl_error);   // portHandler, DXL3_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position3, &dxl_error);  
  if (dxl_comm_result == COMM_SUCCESS) { 
    pos_motors.id[2] = 3;   
    pos_motors.position[2] = position3;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
  }
  
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, 4, ADDR_PRESENT_POSITION, (uint32_t *)&position4, &dxl_error);   // portHandler, DXL4_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position4, &dxl_error);  
  if (dxl_comm_result == COMM_SUCCESS) { 
    pos_motors.id[3] = 4;   
    pos_motors.position[3] = position4;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
  }
  
  get_position_pub.publish(pos_motors);
}

int main(int argc, char ** argv)
{
  // Initialization for use of the motors
  
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  
  // Torque Enable

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }
  
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL3_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL4_ID);
    return -1;
  }
  
  // Start with velocity 15, motor ID 1
  
  //dxl_comm_result = packetHandler->write4ByteTxRx(
  //  portHandler, 1, ADDR_GOAL_VELOCITY, (uint32_t)15, &dxl_error);

  
  // Operating Mode 
  // 1: Velocity 
  // 3: Position   
  
  // Try to change the operating mode by write1ByteTxRx  
  
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_OPERATING_MODE, 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to operating mode for Dynamixel ID %d", DXL1_ID);
    return -1;
  }
  else{
    int8_t op_mode_result = 0;
    packetHandler->read1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING_MODE, (uint8_t *)&op_mode_result, &dxl_error);
    ROS_INFO("Operating mode for Dynamixel ID %d: %d", DXL1_ID, op_mode_result);
  }
  
  // Try to change the operating mode by write2ByteTxRx

  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, DXL2_ID, ADDR_OPERATING_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {    
    ROS_ERROR("Failed to operating mode for Dynamixel ID %d", DXL2_ID);
    return -1;
  }
  else{
    int8_t op_mode_result = 0;
    packetHandler->read1ByteTxRx(portHandler, DXL2_ID, ADDR_OPERATING_MODE, (uint8_t *)&op_mode_result, &dxl_error);
    ROS_INFO("Operating mode for Dynamixel ID %d: %d", DXL2_ID, op_mode_result);
  }
  
  // Try to change the operating mode by write1ByteTxRx
  
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL3_ID, ADDR_OPERATING_MODE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to operating mode for Dynamixel ID %d", DXL3_ID);    
  }
  else{
    int8_t op_mode_result = 0;
    packetHandler->read1ByteTxRx(portHandler, DXL3_ID, ADDR_OPERATING_MODE, (uint8_t *)&op_mode_result, &dxl_error);
    ROS_INFO("Operating mode for Dynamixel ID %d: %d", DXL3_ID, op_mode_result);
  }   

  // Try to change the operating mode by writeTxRx(port, id, address, 1, data_write, error);

  int8_t op_mode_config1 = 1;
  dxl_comm_result = packetHandler->writeTxRx(portHandler, DXL4_ID, ADDR_OPERATING_MODE, 1, (uint8_t *)&op_mode_config1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to operating mode for Dynamixel ID %d", DXL4_ID);
    return -1;
  }
  else{
    int8_t op_mode_result = 0;
    packetHandler->read1ByteTxRx(portHandler, DXL4_ID, ADDR_OPERATING_MODE, (uint8_t *)&op_mode_result, &dxl_error);
    ROS_INFO("Operating mode for Dynamixel ID %d: %d", DXL4_ID, op_mode_result);
  }
  
  // Initialize node, service, subscribers and publishers
  
  ros::init(argc, argv, "read_write_node");
  ros::NodeHandle nh;
  
  ros::ServiceServer get_velocity_srv = nh.advertiseService("/get_velocity", getPresentVelocityCallback);
  ros::Subscriber set_velocity_sub = nh.subscribe("/set_velocity", 10, setVelocityCallback);
  ros::Subscriber set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);
  
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);
  get_position_pub = nh.advertise<dynamixel_sdk_examples::Position>("/position_encoder", 10);
  
  
  while (ros::ok())
  {
    usleep(8*1000);
    ros::spin();
  }
  

  portHandler->closePort();

  return 0;
}
