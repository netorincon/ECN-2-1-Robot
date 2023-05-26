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

// Functions to read keyboard
// Leave as is
int getch(){
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

int kbhit(void){
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

struct MotorGoal{
	std::string id;
	int mode = 3;
	int value = 0;
};

struct MotorState{
	std::string id;
	float position = 0;
	float velocity = 0;
	float torque = 0;
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
			
			// Open port
			if (portHandler->openPort()){
				printf("Succeeded to open the port!\n");
			}
			else{
				printf("Failed to open the port!\n");
				printf("Press any key to terminate...\n");
				getch();
				//return 0;
			}
			
			// Set port baudrate
			if (portHandler->setBaudRate(BAUDRATE)){
				printf("Succeeded to change the baudrate!\n");
			}
			else{
				printf("Failed to change the baudrate!\n");
				printf("Press any key to terminate...\n");
				getch();
				//return 0;
			}
			
			// Enable Dynamixel#1 Torque
			enableTorque(DXL1_ID);
			
			// Enable Dynamixel#2 Torque
			enableTorque(DXL2_ID);
			
			// Enable Dynamixel#3 Torque
			enableTorque(DXL3_ID);
			
			// Enable Dynamixel#4 Torque
			enableTorque(DXL4_ID);

			//// Add parameter storage for Dynamixel#1 present values
			//paramStorageRead(DXL1_ID);
			//if(exitParam){
				//printf("\nParameter storage for DXL1 (return 0)");
				////return 0;
			//}
			
			//// Add parameter storage for Dynamixel#2 present values
			//paramStorageRead(DXL2_ID);
			//if(exitParam){
				//printf("\nParameter storage for DXL2 (return 0)");
				////return 0;
			//}
			
			// Add parameter storage for Dynamixel#3 present values
			paramStorageRead(DXL3_ID);
			if(exitParam){
				printf("\nParameter storage for DXL3 (return 0)");
				//return 0;
			}
			
			// Add parameter storage for Dynamixel#4 present values
			paramStorageRead(DXL4_ID);
			if(exitParam){
				printf("\nParameter storage for DXL4 (return 0)");
				//return 0;
			}
			
			// Get initial values
			getPresentValue();
			if(exitParam){
				printf("\nError in getting present values (return 0)");
				//return 0;
			}
			
			// Initialize in mode 3 (Position Control) with current values
			for(int i = 0; i < 4; i++){
				command[i].id = std::to_string(i+1);
				command[i].mode = 3;
				command[i].value = stateArray[i].position;
			}
				
			//Subscriber for desired joint states		
			motor_command_subscriber = this->create_subscription<control_input::msg::MotorCommand>(
						"motor_cmd",10,std::bind(&motor_state::goalJoints,this,std::placeholders::_1));
			
			//Publisher for present joint states
			motor_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("motor_states",10);
			timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&motor_state::publishJointState,this));
        }
        
        MotorGoal command[4]; //Goal
        MotorState stateArray[4]; //Present
        
        // Initialize PortHandler instance
		// Set the port path
		// Get methods and members of PortHandlerLinux or PortHandlerWindows
		dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        
        //DISABLE TORQUE
		void disableTorque(uint8_t id){
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

    private :
		rclcpp::TimerBase::SharedPtr timer_;
		sensor_msgs::msg::JointState jointState;
        
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_publisher;
        rclcpp::Subscription<control_input::msg::MotorCommand>::SharedPtr motor_command_subscriber;

		// Initialize PacketHandler instance
		// Set the protocol version
		// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
		
		// Initialize GroupBulkWrite instance
		dynamixel::GroupBulkWrite commandPacket = dynamixel::GroupBulkWrite(portHandler, packetHandler);

		// Initialize GroupBulkRead instance
		dynamixel::GroupBulkRead positionPacket = dynamixel::GroupBulkRead(portHandler, packetHandler);
		dynamixel::GroupBulkRead velocityPacket = dynamixel::GroupBulkRead(portHandler, packetHandler);
		dynamixel::GroupBulkRead torquePacket = dynamixel::GroupBulkRead(portHandler, packetHandler);
		
		int dxl_comm_result = COMM_TX_FAIL;             // Communication result
		bool dxl_addparam_result = false;               // addParam result
		bool dxl_getdata_result = false;                // GetParam result
		int dxl1_goal;         							// Goal joint values
		int dxl2_goal;
		int dxl3_goal;
		int dxl4_goal;
		bool exitParam = false;
		
		// Mode selector for control type
		// 0 for torque (current)
		// 1 velocity
		// 3 position (default)
		int dxl_mode[4] = {3,3,3,3};
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
		
	// Position conversion for dynamixel
	int posToPulse(float value){
		return (value*(360/(2*M_PI)))/0.088;
	}
	
	// Velocity conversion for dynamixel
	int velToPulse(float value){
		return (value/(2*M_PI))*60;
	}
	
	// TODO
	// Current (torque) conversion for dynamixel
	int torToPulse(float value){
		return value;
	}
	
	// Position conversion for topic
	float pulseToPos(float value){
		return value*0.088*((2*M_PI)/360);
	}
	
	// Velocity conversion for topic
	float pulseToVel(float value){
		return (value*(2*M_PI))/60;
	}
	
	// TODO
	//Current (torque) conversion for topic
	float pulseToTor(float value){
		return value;
	}
	
	// Method to obtain desired joint states
	void goalJoints(const control_input::msg::MotorCommand::SharedPtr cmd){
		int controlMode;
		
		for(int i=0;i<4;i++){
			//Dynamixel ID
			command[i].id = cmd->cmd.name[i];
			
			if(cmd->mode[i] == "position"){
				controlMode = 3;
				command[i].value = posToPulse(cmd->cmd.position[i]);
			}
			else if(cmd->mode[i] == "velocity"){
				controlMode = 1;
				command[i].value = velToPulse(cmd->cmd.velocity[i]);
			}
			else if(cmd->mode[i] == "torque"){
				controlMode = 0;
				if(cmd->cmd.name[i] != "3" && cmd->cmd.name[i] != "4"){
					command[i].value = torToPulse(cmd->cmd.effort[i]);
				}
			}
			
			//Check for mode change
			if(!command[i].mode || command[i].mode != controlMode){
				command[i].mode = controlMode;
			}
		}
		
		//Check operating mode for every motor and change accordingly
		for(int i=0; i<4; i++){
			if(dxl_mode[i] != command[i].mode){
				changeMode(stoi(command[i].id), command[i].mode);
				if(command[i].mode == 3){
					addressGoal[i] = ADDR_GOAL_POSITION;
					lenSize[i] = LEN_PV;
				}
				else if(command[i].mode == 1){
					addressGoal[i] = ADDR_GOAL_VELOCITY;
					lenSize[i] = LEN_PV;
				}
				else if(command[i].mode == 0){
					addressGoal[i] = ADDR_GOAL_CURRENT;
					lenSize[i] = LEN_CURRENT;
				}
			}
		}

		// Allocate goal position value into byte array
		param1_goal_state[0] = DXL_LOBYTE(DXL_LOWORD((int)command[0].value));
		param1_goal_state[1] = DXL_HIBYTE(DXL_LOWORD((int)command[0].value));
		param1_goal_state[2] = DXL_LOBYTE(DXL_HIWORD((int)command[0].value));
		param1_goal_state[3] = DXL_HIBYTE(DXL_HIWORD((int)command[0].value));
		
		param2_goal_state[0] = DXL_LOBYTE(DXL_LOWORD((int)command[1].value));
		param2_goal_state[1] = DXL_HIBYTE(DXL_LOWORD((int)command[1].value));
		param2_goal_state[2] = DXL_LOBYTE(DXL_HIWORD((int)command[1].value));
		param2_goal_state[3] = DXL_HIBYTE(DXL_HIWORD((int)command[1].value));
		
		param3_goal_state[0] = DXL_LOBYTE(DXL_LOWORD((int)command[2].value));
		param3_goal_state[1] = DXL_HIBYTE(DXL_LOWORD((int)command[2].value));
		param3_goal_state[2] = DXL_LOBYTE(DXL_HIWORD((int)command[2].value));
		param3_goal_state[3] = DXL_HIBYTE(DXL_HIWORD((int)command[2].value));
		
		param4_goal_state[0] = DXL_LOBYTE(DXL_LOWORD((int)command[3].value));
		param4_goal_state[1] = DXL_HIBYTE(DXL_LOWORD((int)command[3].value));
		param4_goal_state[2] = DXL_LOBYTE(DXL_HIWORD((int)command[3].value));
		param4_goal_state[3] = DXL_HIBYTE(DXL_HIWORD((int)command[3].value));

		// Add parameter storage for Dynamixels goal
		paramStorageWrite(DXL1_ID, addressGoal[0], lenSize[0], param1_goal_state);
		if(exitParam){
			printf("Param write 1 (return 0) failed");
			//return 0;
		}
		
		paramStorageWrite(DXL2_ID, addressGoal[1], lenSize[1], param2_goal_state);
		if(exitParam){
			printf("Param write 2 (return 0) failed");
			//return 0;
		}
		
		paramStorageWrite(DXL3_ID, addressGoal[2], lenSize[2], param3_goal_state);
		if(exitParam){
			printf("Param write 3 (return 0) failed");
			//return 0;
		}
		
		paramStorageWrite(DXL4_ID, addressGoal[3], lenSize[3], param4_goal_state);
		if(exitParam){
			printf("Param write 4 (return 0) failed");
			//return 0;
		}

		// Bulkwrite goal joint states
		dxl_comm_result = commandPacket.txPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}

		// Clear bulkwrite parameter storage
		commandPacket.clearParam();
	}
	
	// Method to publish present values of the motors
	void publishJointState(){
		// Check present states
		getPresentValue();
		if(exitParam){
			printf("Could not obtain present value for publisher (return 0)");
			//return 0;
		}
		
		jointState.header.stamp = this->now();
		jointState.name.clear();
		jointState.position.clear();
		jointState.velocity.clear();
		jointState.effort.clear();
		
		for(int i=0;i<4;i++){
			jointState.name.push_back(stateArray[i].id);
			jointState.position.push_back(pulseToPos(stateArray[i].position));
			jointState.velocity.push_back(pulseToVel(stateArray[i].velocity));
			if(stateArray[i].id != "3" && stateArray[i].id != "4"){
				jointState.effort.push_back(pulseToTor(stateArray[i].torque));
			}
		}
		motor_state_publisher->publish(jointState);
	}
	
	//ENABLE TORQUE TO MOTORS
	void enableTorque(uint8_t id){
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

	//DEFINE OR CHANGE OPERATING MODE OF MOTORS
	void changeMode(uint8_t id, uint8_t mode){
		disableTorque(id);
		
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
		
		enableTorque(id);
	}

	// Add parameter storage for all dynamixels
	void paramStorageRead(uint8_t id){
		// Add parameter storage for Dynamixel#id present position
		dxl_addparam_result = positionPacket.addParam(id, ADDR_PRESENT_POSITION, LEN_PV);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (position)", id);
			exitParam = true;
			//return;
		}
		
		// Add parameter storage for Dynamixel#id present velocity
		//dxl_addparam_result = velocityPacket.addParam(id, ADDR_PRESENT_VELOCITY, LEN_PV);
		//if (dxl_addparam_result != true){
			//fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (velocity)", id);
			//exitParam = true;
			////return;
		//}
		
		//Motors 3 and 4 (rotational) do not have torque control, there's no need to read the value
		//However, we could technically read present load in the motors
		//In said case, remove the if condition which skips id 3 and 4
		//if(id != 3 && id != 4){
			//// Add parameter storage for Dynamixel#id present torque
			//dxl_addparam_result = torquePacket.addParam(id, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			//if (dxl_addparam_result != true){
				//fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (torque)", id);
				//exitParam = true;
				////return;
			//}
		//}
	}

	void readAvailable(dynamixel::GroupBulkRead &groupBulkRead, uint8_t id, int address, int len){
		dxl_getdata_result = groupBulkRead.isAvailable(id, address, len);
		if (dxl_getdata_result != true){
			fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", id);
			exitParam = true;
			//return;
		}
	}

	void getPresentValue(){
		// Bulkread present state
		dxl_comm_result = positionPacket.txRxPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		//else if (positionPacket.getError(DXL1_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		//else if (positionPacket.getError(DXL2_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		else if (positionPacket.getError(DXL3_ID, &dxl_error)){
			printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(dxl_error));
		}
		else if (positionPacket.getError(DXL4_ID, &dxl_error)){
			printf("[ID:%03d] %s\n", DXL4_ID, packetHandler->getRxPacketError(dxl_error));
		}
		
		//readAvailable(positionPacket, DXL1_ID, ADDR_PRESENT_POSITION, LEN_PV);
		//if(exitParam){
			////return;
		//}
		
		//readAvailable(positionPacket, DXL2_ID, ADDR_PRESENT_POSITION, LEN_PV);
		//if(exitParam){
			////return;
		//}
		
		readAvailable(positionPacket, DXL3_ID, ADDR_PRESENT_POSITION, LEN_PV);
		if(exitParam){
			//return;
		}
		
		readAvailable(positionPacket, DXL4_ID, ADDR_PRESENT_POSITION, LEN_PV);
		if(exitParam){
			//return;
		}
		
		// Store initial values
		//stateArray[0].id = "1";
		//stateArray[0].position = positionPacket.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PV);
		//stateArray[1].id = "2";
		//stateArray[1].position = positionPacket.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PV);
		stateArray[2].id = "3";
		stateArray[2].position = positionPacket.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PV);
		stateArray[3].id = "4";
		stateArray[3].position = positionPacket.getData(DXL4_ID, ADDR_PRESENT_POSITION, LEN_PV);
		
		//dxl_comm_result = velocityPacket.txRxPacket();
		//if (dxl_comm_result != COMM_SUCCESS){
			//printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		//}
		//else if (velocityPacket.getError(DXL1_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		//else if (velocityPacket.getError(DXL2_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		//else if (velocityPacket.getError(DXL3_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL3_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		//else if (velocityPacket.getError(DXL4_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL4_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		
		//readAvailable(velocityPacket, DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		//if(exitParam){
			////return;
		//}
		
		//readAvailable(velocityPacket, DXL2_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		//if(exitParam){
			////return;
		//}
		
		//readAvailable(velocityPacket, DXL3_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		//if(exitParam){
			////return;
		//}
		
		//readAvailable(velocityPacket, DXL4_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		//if(exitParam){
			////return;
		//}
		
		//stateArray[0].velocity = velocityPacket.getData(DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		//stateArray[1].velocity = velocityPacket.getData(DXL2_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		//stateArray[2].velocity = velocityPacket.getData(DXL3_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		//stateArray[3].velocity = velocityPacket.getData(DXL4_ID, ADDR_PRESENT_VELOCITY, LEN_PV);
		
		//dxl_comm_result = torquePacket.txRxPacket();
		//if (dxl_comm_result != COMM_SUCCESS){
			//printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		//}
		//else if (torquePacket.getError(DXL1_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		//else if (torquePacket.getError(DXL2_ID, &dxl_error)){
			//printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
		//}
		
		//readAvailable(torquePacket, DXL1_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);
		//if(exitParam){
			////return;
		//}
		
		//readAvailable(torquePacket, DXL2_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);
		//if(exitParam){
			////return;
		//}
		
		//stateArray[0].torque = torquePacket.getData(DXL1_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);
		//stateArray[1].torque = torquePacket.getData(DXL2_ID, ADDR_PRESENT_CURRENT, LEN_CURRENT);

		//printf("[ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \t Present Torque : %.2f \n [ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \t Present Torque : %.2f \n [ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \n [ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \n", DXL1_ID, stateArray[0].position, stateArray[0].velocity, stateArray[0].torque, DXL2_ID, stateArray[1].position, stateArray[1].velocity, stateArray[1].torque, DXL3_ID, stateArray[2].position, stateArray[2].velocity, DXL4_ID, stateArray[3].position, stateArray[3].velocity);
		
		printf("\n[ID:%03d] \n Present Position : %.2f \t\n [ID:%03d] \n Present Position : %.2f \t\n", DXL1_ID, stateArray[0].position, DXL2_ID, stateArray[1].position);
	}

	void paramStorageWrite(uint8_t id, int address, int len, uint8_t param_goal_position[4]){
		dxl_addparam_result = commandPacket.addParam(id, address, len, param_goal_position);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] commandPacket addparam failed", id);
			exitParam = true;
			//return;
		}
	}
};

int main(int argc, char** argv)
{	
	rclcpp::init(argc, argv);
	auto node = std::make_shared<motor_state>();
	//node = std::make_shared<motor_state>(rclcpp::NodeOptions{});
	rclcpp::spin(node);
	
	// Make sure to disable torque and close port
	node->disableTorque(DXL1_ID);
	node->disableTorque(DXL2_ID);
	node->disableTorque(DXL3_ID);
	node->disableTorque(DXL4_ID);
	node->portHandler->closePort();
	
	rclcpp::shutdown();
	return 0;
}
