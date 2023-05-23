#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>

// Used to publish current state to topic
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_input/msg/motor_command.hpp>

// Uses Dynamixel SDK library
#include "DynamixelSDK.h"

// Control table address

// EEPROM Area
// Data in EEPROM area can only be written when torque value (ADDR_TORQUE_ENABLE) is 0
#define ADDR_OPERATING_MODE				11

// RAM Area
// Rotation motors DO NOT have torque control
#define ADDR_TORQUE_ENABLE				64
#define ADDR_GOAL_CURRENT				102 // Does NOT exist in Rot motors
#define ADDR_GOAL_VELOCITY				104
#define ADDR_GOAL_POSITION				116
#define ADDR_PRESENT_CURRENT			126 // Represents "Present Load" in Rot motors
#define ADDR_PRESENT_VELOCITY			128
#define ADDR_PRESENT_POSITION			132

// Data Byte Length
#define LEN_CURRENT						2
#define LEN_PV							4 //Position and Velocity

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
// Spinning motors
#define DXL1_ID                         1
#define DXL2_ID                         2
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

struct MotorGoal{
	std::string id;
	int mode;
	float value;
};

struct MotorState{
	std::string id;
	float position;
	float velocity;
	float torque;
};

struct DxlState{
	uint32_t position;
	uint32_t velocity;
	uint16_t torque;
};

class motor_state : public rclcpp::Node
{
    public :
        //motor_state(rclcpp::NodeOptions options) : Node("motor_state", options){
        motor_state() : Node("motor_state"){			
			//Subscriber for desired joint states		
			motor_command_subscriber = this->create_subscription<control_input::msg::MotorCommand>(
						"motor_cmd",10,std::bind(&motor_state::goalJoints,this,std::placeholders::_1));
			
			//Publisher for present joint states
			motor_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("motor_states",10);
			timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&motor_state::publishJointState,this));
			
        }
        
        MotorGoal command[4]; //Goal
        MotorState stateArray[4]; //Present

    private :
		rclcpp::TimerBase::SharedPtr timer_;
		sensor_msgs::msg::JointState jointState;
        
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_publisher;
        rclcpp::Subscription<control_input::msg::MotorCommand>::SharedPtr motor_command_subscriber;
	
	// Method to obtain desired joint states
	void goalJoints(const control_input::msg::MotorCommand::SharedPtr cmd){
		int controlMode;
		
		for(int i=0;i<4;i++){
			//Dynamixel ID
			command[i].id = cmd->cmd.name[i];
			
			if(cmd->mode[i] == "position"){
				controlMode = 3;
				command[i].value = cmd->cmd.position[i];
			}
			else if(cmd->mode[i] == "velocity"){
				controlMode = 1;
				command[i].value = cmd->cmd.velocity[i];
			}
			else if(cmd->mode[i] == "torque"){
				controlMode = 0;
				if(cmd->cmd.name[i] != "3" && cmd->cmd.name[i] != "4"){
					command[i].value = cmd->cmd.effort[i];
				}
			}
			
			//Check for mode change
			if(!command[i].mode || command[i].mode != controlMode){
				command[i].mode = controlMode;
			}
		}
	}
	
	// Method to publish present values of the motors
	void publishJointState(){
		jointState.header.stamp = this->now();
		for(int i=0;i<4;i++){
			jointState.name[i] = stateArray[i].id;
			jointState.position[i] = stateArray[i].position;
			jointState.velocity[i] = stateArray[i].velocity;
			if(stateArray[i].id != "3" && stateArray[i].id != "4"){
				jointState.effort[i] = stateArray[i].torque;
			}
		}
		motor_state_publisher->publish(jointState);
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

//ENABLE TORQUE TO MOTORS
void enableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint8_t &dxl_error){
	int dxl_comm_result = COMM_TX_FAIL;
	
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0){
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else{
		printf("Dynamixel#%d has been successfully connected \n", id);
	}
}

//DISABLE TORQUE
void disableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint8_t &dxl_error){
	int dxl_comm_result = COMM_TX_FAIL;
	
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0){
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else{
		printf("Dynamixel#%d has been successfully disconnected \n", id);
	}
}

//DEFINE OR CHANGE OPERATING MODE OF MOTORS
void changeMode(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint8_t mode, uint8_t &dxl_error){
	int dxl_comm_result = COMM_TX_FAIL;
	
	disableTorque(portHandler,packetHandler,id,dxl_error);
	
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, mode, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0){
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	//else{
		//printf("Dynamixel#%d has been successfully disconnected \n", id);
	//}
	
	enableTorque(portHandler,packetHandler,id,dxl_error);
}

// Add parameter storage for all dynamixels
void paramStorageRead(dynamixel::GroupBulkRead &groupBulkRead, uint8_t id, bool &exitParam){
	bool dxl_addparam_result = false;
	
	// Add parameter storage for Dynamixel#id present position
	dxl_addparam_result = groupBulkRead.addParam(id, ADDR_PRESENT_POSITION, LEN_PV);
	if (dxl_addparam_result != true){
		fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (position)", id);
		exitParam = true;
		return;
	}
	
	// Add parameter storage for Dynamixel#id present velocity
	dxl_addparam_result = groupBulkRead.addParam(id, ADDR_PRESENT_VELOCITY, LEN_PV);
	if (dxl_addparam_result != true){
		fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (velocity)", id);
		exitParam = true;
		return;
	}
	
	//Motors 3 and 4 (rotational) do not have torque control, there's no need to read the value
	//However, we could technically read present load in the motors
	//In said case, remove the if condition which skips id 3 and 4
	if(id != 3 && id != 4){
		// Add parameter storage for Dynamixel#id present torque
		dxl_addparam_result = groupBulkRead.addParam(id, ADDR_PRESENT_CURRENT, LEN_CURRENT);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (torque)", id);
			exitParam = true;
			return;
		}
	}
}

void readAvailable(dynamixel::GroupBulkRead &groupBulkRead, uint8_t id, int address, int len, bool &exitParam){
	bool dxl_getdata_result = false;
	
	dxl_getdata_result = groupBulkRead.isAvailable(id, address, len);
	if (dxl_getdata_result != true){
		fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", id);
		exitParam = true;
		return;
	}
}

void getPresentValue(dynamixel::PacketHandler *packetHandler, dynamixel::GroupBulkRead &groupBulkRead, uint8_t &dxl_error, bool &exitParam){
	// Bulkread present state
	int dxl_comm_result;
	
	dxl_comm_result = groupBulkRead.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (groupBulkRead.getError(DXL1_ID, &dxl_error)){
		printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
	}
	else if (groupBulkRead.getError(DXL2_ID, &dxl_error)){
		printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
	}
	else if (groupBulkRead.getError(DXL3_ID, &dxl_error)){
		printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(dxl_error));
	}
	else if (groupBulkRead.getError(DXL4_ID, &dxl_error)){
		printf("[ID:%03d] %s\n", DXL4_ID, packetHandler->getRxPacketError(dxl_error));
	}

	// Check if groupbulkread data of Dynamixel#1 is available
	readAvailable(groupBulkRead, DXL1_ID, ADDR_PRESENT_POSITION, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL1_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT, exitParam);
	if(exitParam){
		return;
	}
	
	// Check if groupbulkread data of Dynamixel#2 is available
	readAvailable(groupBulkRead, DXL2_ID, ADDR_PRESENT_POSITION, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL2_ID, ADDR_PRESENT_VELOCITY, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL2_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT, exitParam);
	if(exitParam){
		return;
	}
	
	// Check if groupbulkread data of Dynamixel#3 is available
	readAvailable(groupBulkRead, DXL3_ID, ADDR_PRESENT_POSITION, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL3_ID, ADDR_PRESENT_VELOCITY, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL3_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT, exitParam);
	if(exitParam){
		return;
	}
	
	// Check if groupbulkread data of Dynamixel#4 is available
	readAvailable(groupBulkRead, DXL4_ID, ADDR_PRESENT_POSITION, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL4_ID, ADDR_PRESENT_VELOCITY, LEN_PV, exitParam);
	if(exitParam){
		return;
	}
	readAvailable(groupBulkRead, DXL4_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT, exitParam);
	if(exitParam){
		return;
	}
}

void paramStorageWrite(dynamixel::GroupBulkWrite &groupBulkWrite, uint8_t id, int address, int len, uint8_t param_goal_position[4], bool &exitParam){
	bool dxl_addparam_result = false;
	
	dxl_addparam_result = groupBulkWrite.addParam(id, address, len, param_goal_position);
	if (dxl_addparam_result != true){
		fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", id);
		exitParam = true;
		return;
	}
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
	
	// Initialize GroupBulkWrite instance
	dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

	// Initialize GroupBulkRead instance
	dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);
	
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	//bool dxl_addparam_result = false;               // addParam result
	//bool dxl_getdata_result = false;                // GetParam result
	int dxl1_goal;         							// Goal joint values
	int dxl2_goal;
	int dxl3_goal;
	int dxl4_goal;
	bool exitParam = false;
	
	// Mode selector for control type
	// 0 for torque (current)
	// 1 velocity
	// 3 position (default)
	int dxl_mode[4];
	int addressGoal[4] = {ADDR_GOAL_POSITION,ADDR_GOAL_POSITION,ADDR_GOAL_POSITION,ADDR_GOAL_POSITION};
	int lenSize[4] = {LEN_PV,LEN_PV,LEN_PV,LEN_PV}; // LEN_PV or LEN_CURRENT

	uint8_t dxl_error = 0;                          // Dynamixel error
	uint8_t param1_goal_state[4];					// Goal state
	uint8_t param2_goal_state[4];
	uint8_t param3_goal_state[4];
	uint8_t param4_goal_state[4];
	DxlState dxl1_present_state;					// Present position (4 bytes)
	DxlState dxl2_present_state;
	DxlState dxl3_present_state;
	DxlState dxl4_present_state;
	
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
	enableTorque(portHandler,packetHandler,DXL1_ID,dxl_error);
	
	// Enable Dynamixel#2 Torque
	enableTorque(portHandler,packetHandler,DXL2_ID,dxl_error);
	
	// Enable Dynamixel#3 Torque
	enableTorque(portHandler,packetHandler,DXL3_ID,dxl_error);
	
	// Enable Dynamixel#4 Torque
	enableTorque(portHandler,packetHandler,DXL4_ID,dxl_error);

	// Add parameter storage for Dynamixel#1 present values
	paramStorageRead(groupBulkRead, DXL1_ID, exitParam);
	if(exitParam){
		return 0;
	}
	
	// Add parameter storage for Dynamixel#2 present values
	paramStorageRead(groupBulkRead, DXL2_ID, exitParam);
	if(exitParam){
		return 0;
	}
	
	// Add parameter storage for Dynamixel#3 present values
	paramStorageRead(groupBulkRead, DXL3_ID, exitParam);
	if(exitParam){
		return 0;
	}
	
	// Add parameter storage for Dynamixel#4 present values
	paramStorageRead(groupBulkRead, DXL4_ID, exitParam);
	if(exitParam){
		return 0;
	}
	
	// Get initial values
	getPresentValue(packetHandler, groupBulkRead, dxl_error, exitParam);
	if(exitParam){
		return 0;
	}
	
	dxl1_present_state.position = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PV);
	dxl1_present_state.velocity = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
	dxl1_present_state.torque = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);
	
	dxl2_present_state.position = groupBulkRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PV);
	dxl2_present_state.velocity = groupBulkRead.getData(DXL2_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
	dxl2_present_state.torque = groupBulkRead.getData(DXL2_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);
	
	dxl3_present_state.position = groupBulkRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PV);
	dxl3_present_state.velocity = groupBulkRead.getData(DXL3_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
	
	dxl4_present_state.position = groupBulkRead.getData(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PV);
	dxl4_present_state.velocity = groupBulkRead.getData(DXL4_ID, ADDR_PRESENT_VELOCITY, LEN_PV);

	printf("[ID:%03d] \n Present Position : %d \t Present Velocity : %d \t Present Torque : %d \n [ID:%03d] \n Present Position : %d \t Present Velocity : %d \t Present Torque : %d \n [ID:%03d] \n Present Position : %d \t Present Velocity : %d \n [ID:%03d] \n Present Position : %d \t Present Velocity : %d \n", DXL1_ID, dxl1_present_state.position, dxl1_present_state.velocity, dxl1_present_state.torque, DXL2_ID, dxl2_present_state.position, dxl2_present_state.velocity, dxl2_present_state.torque, DXL3_ID, dxl3_present_state.position, dxl3_present_state.velocity, DXL4_ID, dxl4_present_state.position, dxl4_present_state.velocity);
	
	// Initialize node
	rclcpp::init(argc, argv);
	auto node = std::make_shared<motor_state>();
	
	// Store initial values
	node->stateArray[0].id = "1";
	node->stateArray[0].position = dxl1_present_state.position;
	node->stateArray[0].velocity = dxl1_present_state.velocity;
	node->stateArray[0].torque = dxl1_present_state.torque;
	
	node->stateArray[1].id = "2";
	node->stateArray[1].position = dxl2_present_state.position;
	node->stateArray[1].velocity = dxl2_present_state.velocity;
	node->stateArray[1].torque = dxl2_present_state.torque;
	
	node->stateArray[2].id = "3";
	node->stateArray[2].position = dxl3_present_state.position;
	node->stateArray[2].velocity = dxl3_present_state.velocity;
	
	node->stateArray[3].id = "4";
	node->stateArray[3].position = dxl4_present_state.position;
	node->stateArray[3].velocity = dxl4_present_state.velocity;
	
	while(rclcpp::ok()){
		//printf("Press any key to continue! (or press ESC to quit!)\n");
		//if (getch() == ESC_ASCII_VALUE)
			//break;
		
		//Verify is similar to spin_once in ROS1
		rclcpp::spin_some(node);
		
		//Check operating mode for every motor and change accordingly
		for(int i=0; i<4; i++){
			if(dxl_mode[i] != node->command[i].mode){
				changeMode(portHandler, packetHandler, stoi(node->command[i].id), node->command[i].mode, dxl_error);
				if(node->command[i].mode == 3){
					addressGoal[i] = ADDR_GOAL_POSITION;
					lenSize[i] = LEN_PV;
				}
				else if(node->command[i].mode == 1){
					addressGoal[i] = ADDR_GOAL_VELOCITY;
					lenSize[i] = LEN_PV;
				}
				else if(node->command[i].mode == 0){
					addressGoal[i] = ADDR_GOAL_CURRENT;
					lenSize[i] = LEN_CURRENT;
				}
			}
		}
		
		//Obtain goal joints
		dxl1_goal = (int)node->command[0].value;
		dxl2_goal = (int)node->command[1].value;
		dxl3_goal = (int)node->command[2].value;
		dxl4_goal = (int)node->command[3].value;

		// Allocate goal position value into byte array
		param1_goal_state[0] = DXL_LOBYTE(DXL_LOWORD(dxl1_goal));
		param1_goal_state[1] = DXL_HIBYTE(DXL_LOWORD(dxl1_goal));
		param1_goal_state[2] = DXL_LOBYTE(DXL_HIWORD(dxl1_goal));
		param1_goal_state[3] = DXL_HIBYTE(DXL_HIWORD(dxl1_goal));
		
		param2_goal_state[0] = DXL_LOBYTE(DXL_LOWORD(dxl2_goal));
		param2_goal_state[1] = DXL_HIBYTE(DXL_LOWORD(dxl2_goal));
		param2_goal_state[2] = DXL_LOBYTE(DXL_HIWORD(dxl2_goal));
		param2_goal_state[3] = DXL_HIBYTE(DXL_HIWORD(dxl2_goal));
		
		param3_goal_state[0] = DXL_LOBYTE(DXL_LOWORD(dxl3_goal));
		param3_goal_state[1] = DXL_HIBYTE(DXL_LOWORD(dxl3_goal));
		param3_goal_state[2] = DXL_LOBYTE(DXL_HIWORD(dxl3_goal));
		param3_goal_state[3] = DXL_HIBYTE(DXL_HIWORD(dxl3_goal));
		
		param4_goal_state[0] = DXL_LOBYTE(DXL_LOWORD(dxl4_goal));
		param4_goal_state[1] = DXL_HIBYTE(DXL_LOWORD(dxl4_goal));
		param4_goal_state[2] = DXL_LOBYTE(DXL_HIWORD(dxl4_goal));
		param4_goal_state[3] = DXL_HIBYTE(DXL_HIWORD(dxl4_goal));

		// Add parameter storage for Dynamixels goal
		paramStorageWrite(groupBulkWrite, DXL1_ID, addressGoal[0], lenSize[0], param1_goal_state, exitParam);
		if(exitParam){
			return 0;
		}
		
		paramStorageWrite(groupBulkWrite, DXL2_ID, addressGoal[1], lenSize[1], param2_goal_state, exitParam);
		if(exitParam){
			return 0;
		}
		
		paramStorageWrite(groupBulkWrite, DXL3_ID, addressGoal[2], lenSize[2], param3_goal_state, exitParam);
		if(exitParam){
			return 0;
		}
		
		paramStorageWrite(groupBulkWrite, DXL4_ID, addressGoal[3], lenSize[3], param4_goal_state, exitParam);
		if(exitParam){
			return 0;
		}

		// Bulkwrite goal joint states
		dxl_comm_result = groupBulkWrite.txPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}

		// Clear bulkwrite parameter storage
		groupBulkWrite.clearParam();
		
		// Check present states
		getPresentValue(packetHandler, groupBulkRead, dxl_error, exitParam);
		if(exitParam){
			return 0;
		}
		
		// Get present values
		dxl1_present_state.position = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PV);
		dxl1_present_state.velocity = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		dxl1_present_state.torque = groupBulkRead.getData(DXL1_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);
		
		dxl2_present_state.position = groupBulkRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PV);
		dxl2_present_state.velocity = groupBulkRead.getData(DXL2_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		dxl2_present_state.torque = groupBulkRead.getData(DXL2_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);
		
		dxl3_present_state.position = groupBulkRead.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PV);
		dxl3_present_state.velocity = groupBulkRead.getData(DXL3_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		
		dxl4_present_state.position = groupBulkRead.getData(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PV);
		dxl4_present_state.velocity = groupBulkRead.getData(DXL4_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		
		printf("[ID:%03d] \n Present Position : %d \t Present Velocity : %d \t Present Torque : %d \n [ID:%03d] \n Present Position : %d \t Present Velocity : %d \t Present Torque : %d \n [ID:%03d] \n Present Position : %d \t Present Velocity : %d \n [ID:%03d] \n Present Position : %d \t Present Velocity : %d \n", DXL1_ID, dxl1_present_state.position, dxl1_present_state.velocity, dxl1_present_state.torque, DXL2_ID, dxl2_present_state.position, dxl2_present_state.velocity, dxl2_present_state.torque, DXL3_ID, dxl3_present_state.position, dxl3_present_state.velocity, DXL4_ID, dxl4_present_state.position, dxl4_present_state.velocity);
		
		node->stateArray[0].id = "1";
		node->stateArray[0].position = dxl1_present_state.position;
		node->stateArray[0].velocity = dxl1_present_state.velocity;
		node->stateArray[0].torque = dxl1_present_state.torque;
		
		node->stateArray[1].id = "2";
		node->stateArray[1].position = dxl2_present_state.position;
		node->stateArray[1].velocity = dxl2_present_state.velocity;
		node->stateArray[1].torque = dxl2_present_state.torque;
		
		node->stateArray[2].id = "3";
		node->stateArray[2].position = dxl3_present_state.position;
		node->stateArray[2].velocity = dxl3_present_state.velocity;
		
		node->stateArray[3].id = "4";
		node->stateArray[3].position = dxl4_present_state.position;
		node->stateArray[3].velocity = dxl4_present_state.velocity;
	}

	// Disable Dynamixel#1 Torque
	disableTorque(portHandler,packetHandler,DXL1_ID,dxl_error);
	
	// Disable Dynamixel#2 Torque
	disableTorque(portHandler,packetHandler,DXL2_ID,dxl_error);
	
	// Disable Dynamixel#3 Torque
	disableTorque(portHandler,packetHandler,DXL3_ID,dxl_error);
	
	// Disable Dynamixel#4 Torque
	disableTorque(portHandler,packetHandler,DXL4_ID,dxl_error);
	
	// Close port
	portHandler->closePort();	
	
	//rclcpp::init(argc, argv);
	//auto node = std::make_shared<motor_state>();
	//rclcpp::spin(node);
	//rclcpp::spin(std::make_shared<transform_broadcaster>(rclcpp::NodeOptions{}));
	
	rclcpp::shutdown();
	return 0;
}
