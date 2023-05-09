#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
//#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>

// Used to publish current state to topic
#include <sensor_msgs/msg/joint_state.hpp>

// Uses Dynamixel SDK library
#include "DynamixelSDK.h"

// Control table address
//#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
//#define ADDR_PRO_GOAL_POSITION          596
//#define ADDR_PRO_PRESENT_POSITION       611

// EEPROM Area
// Data in EEPROM area can only be written when torque value (64) is 0
#define ADDR_OPERATING_MODE				11

// RAM Area
// Rotation motors DO NOT have torque control
#define ADDR_TORQUE_ENABLE				64
#define ADDR_GOAL_CURRENT				102 // Does NOT exist in Rot motors
#define ADDR_GOAL_VELOCITY				104
#define ADDR_GOAL_POSITION				116
#define ADDR_PRESENT_CURRENT			126 // Represents Present Load in Rot motors
#define ADDR_PRESENT_VELOCITY			128
#define ADDR_PRESENT_POSITION			132

// Data Byte Length
//#define LEN_PRO_GOAL_POSITION           4
//#define LEN_PRO_PRESENT_POSITION        4

#define LEN_POSITION					4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
// Spinning motors
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
// Rotation motors
#define DXL3_ID                         3
#define DXL4_ID                         4

//#define BAUDRATE                        57600
//#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define BAUDRATE						1000000
#define DEVICENAME						"/dev/ttyACM0"


#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

//ESC key
#define ESC_ASCII_VALUE                 0x1b


class motor_state : public rclcpp::Node
{
    public :
        motor_state(rclcpp::NodeOptions options) : Node("motor_state", options)
        {


            //init transform broadcastor
            //tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            //init subscriber
            //subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                            //"cmd_robot", 10, std::bind(&transform_broadcaster::make_transforms, this, std::placeholders::_1));




        }
        float tt = 0, x = 0, y = 0;
        float a=0.05;

    private :
        geometry_msgs::msg::TransformStamped transform_stamped_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
        //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;








void make_transforms(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {

        float Um = msg->data[0];
        float bt1 = msg->data[1];
        float bt2 = msg->data[2];



        float tt_dot=(tan(bt2)-tan(bt1))*Um;
        float x_dot=(a*cos(tt)*(tan(bt1)+tan(bt2))+2*a*sin(tt))*Um;
        float y_dot=(a*sin(tt)*(tan(bt1)+tan(bt2))-2*a*sin(tt))*Um;

        float frequency = 20; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;

        tt += tt_dot*period;
        x += x_dot*period;
        y += y_dot*period;


        transform_stamped_.header.stamp = this->now();
        transform_stamped_.header.frame_id = "world"; // Nom du repère fixe
        transform_stamped_.child_frame_id = "chassis"; // Nom du repère du robot
        transform_stamped_.transform.translation.x += x_dot;
        transform_stamped_.transform.translation.y += y_dot;
        transform_stamped_.transform.translation.z += 0;
        transform_stamped_.transform.rotation.x += 0;
        transform_stamped_.transform.rotation.y += 0;
        transform_stamped_.transform.rotation.z += tt;
        transform_stamped_.transform.rotation.w = 1.0;

        //tf_broadcaster_->sendTransform(transform_stamped_);
    }





};

// Functions to read keyboard
// Leave as is
int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}


int main(int argc, char** argv)
{
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

	// Initialize Groupsyncread instance for Present Position
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;               // Communication result
	bool dxl_addparam_result = false;                 // addParam result
	bool dxl_getdata_result = false;                  // GetParam result
	int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};  // Goal position

	uint8_t dxl_error = 0;                            // Dynamixel error
	uint8_t param_goal_position[4];
	int32_t dxl1_present_position = 0, dxl2_present_position = 0;                         // Present position

	// Open port
	if (portHandler->openPort()){
		printf("Succeeded to open the port!\n");
	}
	else{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE)){
		printf("Succeeded to change the baudrate!\n");
	}
	else{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Enable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0){
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else{
		printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
	}

	// Enable Dynamixel#2 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0){
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else{
		printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
	}

	// Add parameter storage for Dynamixel#1 present position value
	dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
	if (dxl_addparam_result != true){
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
		return 0;
	}

	// Add parameter storage for Dynamixel#2 present position value
	dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
	if (dxl_addparam_result != true){
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
		return 0;
	}

	while(1){
		printf("Press any key to continue! (or press ESC to quit!)\n");
		if (getch() == ESC_ASCII_VALUE)
			break;

		// Allocate goal position value into byte array
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

		// Add Dynamixel#1 goal position value to the Syncwrite storage
		dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
			return 0;
		}

		// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
			return 0;
		}

		// Syncwrite goal position
		dxl_comm_result = groupSyncWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}

		// Clear syncwrite parameter storage
		groupSyncWrite.clearParam();

		do{
			// Syncread present position
			dxl_comm_result = groupSyncRead.txRxPacket();
			if (dxl_comm_result != COMM_SUCCESS){
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (groupSyncRead.getError(DXL1_ID, &dxl_error)){
				printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
			}
			else if (groupSyncRead.getError(DXL2_ID, &dxl_error)){
				printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
			}

			// Check if groupsyncread data of Dynamixel#1 is available
			dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
			if (dxl_getdata_result != true){
				fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
				return 0;
			}

			// Check if groupsyncread data of Dynamixel#2 is available
			dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
			if (dxl_getdata_result != true){
				fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
				return 0;
			}

			// Get Dynamixel#1 present position value
			dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

			// Get Dynamixel#2 present position value
			dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

			printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position[index], dxl2_present_position);

		}while((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

		// Change goal position
		if (index == 0){
			index = 1;
		}
		else{
			index = 0;
		}
	}

	// Disable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0){
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// Disable Dynamixel#2 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0){
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// Close port
	portHandler->closePort();
	
	
	rclcpp::init(argc, argv);
	//rclcpp::spin(std::make_shared<transform_broadcaster>(rclcpp::NodeOptions{}));
	rclcpp::shutdown();
	return 0;
}


