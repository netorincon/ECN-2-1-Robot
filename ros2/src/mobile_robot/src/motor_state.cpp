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

//#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
//#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
//#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

//ESC key
//#define ESC_ASCII_VALUE                 0x1b

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

class motor_state : public rclcpp::Node
{
    public :
        //motor_state(rclcpp::NodeOptions options) : Node("motor_state", options){
        motor_state() : Node("motor_state"){
			
			// Open port
			if (portHandler->openPort()){
				printf("Succeeded to open the port!\n");
				
				// Set port baudrate
				if (portHandler->setBaudRate(BAUDRATE)){
					printf("Succeeded to change the baudrate!\n");
					
					// Enable Dynamixel#1 Torque
					enableTorque(DXL1_ID);
					
					// Enable Dynamixel#2 Torque
					enableTorque(DXL2_ID);
					
					// Enable Dynamixel#3 Torque
					enableTorque(DXL3_ID);
					
					// Enable Dynamixel#4 Torque
					enableTorque(DXL4_ID);
					
					assignReadParam();
						
					// Initialize in mode 3 (Position Control) with current values
					for(int i = 0; i < 4; i++){
						command[i].id = std::to_string(i+1);
						command[i].mode = 3;
						command[i].value = stateArray[i].position;
						
						changeMode(stoi(command[i].id), command[i].mode);
					}
						
					//Subscriber for desired joint states		
					motor_command_subscriber = this->create_subscription<control_input::msg::MotorCommand>(
								"motor_cmd",10,std::bind(&motor_state::goalJoints,this,std::placeholders::_1));
					
					//Publisher for present joint states
					motor_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("motor_states",10);
					timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&motor_state::publishJointState,this));
				}
				else{
					printf("Failed to change the baudrate!\n");
					printf("Press CTRL + C terminate...\n");
				}
			}
			else{
				printf("Failed to open the port!\n");
				printf("Press CTRL + C terminate...\n");
			}
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
				printf("Dynamixel#%d's torque has been disabled \n", id);
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
		bool exitParam = false;
		
		int addressGoal[4] = {ADDR_GOAL_POSITION,ADDR_GOAL_POSITION,ADDR_GOAL_POSITION,ADDR_GOAL_POSITION};
		int lenSize[4] = {LEN_PV,LEN_PV,LEN_PV,LEN_PV}; // LEN_PV or LEN_CURRENT

		uint8_t dxl_error = 0;                          // Dynamixel error
		uint8_t param1_goal_state[4];					// Goal state
		uint8_t param2_goal_state[4];
		uint8_t param3_goal_state[4];
		uint8_t param4_goal_state[4];
		
	// Position conversion for orientation motors
	// 2048 is the wheel's 0
	// + PI we need to add 2048
	// - PI we need to substract 2048
	int posToPulse(float value){
		int pos = 2048 + ((value * 2048) / M_PI);
		if(pos < 0 || pos > 4095){
			pos = 0;
		}
		return pos;
	}
	
	// Velocity conversion for spinning motors
	// Increments are of 0.229 rpm in both motors
	int velToPulse(float value){
		return ((value * 60) / (2 * M_PI)) / 0.229;
	}
	
	// TODO
	// Torque conversion for dynamixel
	int torToPulse(float value){
		return value;
	}
	
	// Position conversion for topic
	// Change orientation motors position to radians
	float pulseToPos(float value){
		return ((((int)value) % 4096) - 2048)*(M_PI / 2048);
	}
	
	// Velocity conversion for topic
	// Change motor value to rad/s
	float pulseToVel(float value){
		return ((((int)value) % 1024) * 0.229 * 2 * M_PI) / 60;
	}
	
	// TODO
	// Torque conversion for topic
	float pulseToTor(float value){
		return value;
	}
	
	// Method to obtain desired joint states
	void goalJoints(const control_input::msg::MotorCommand::SharedPtr cmd){
		int controlMode;
		
		for(int i=0;i<4;i++){
			//Dynamixel ID
			command[i].id = cmd->cmd.name[i];
			
			//
			printf("Recibo ESTA id : %d \n", stoi(command[i].id));
			
			if(cmd->mode[i] == "position"){
				
				//
				std::cout << "Este valor de posicion : " << +cmd->cmd.position[i] << std::endl;
				//printf("Este valor de posicion: %d",cmd->cmd.position[i]);
				
				controlMode = 3;
				command[i].value = posToPulse(cmd->cmd.position[i]);
			}
			else if(cmd->mode[i] == "velocity"){
				
				//
				std::cout << "Este valor de posicion : " << +cmd->cmd.velocity[i] << std::endl;
				//printf("Este valor de posicion: %d",cmd->cmd.velocity[i]);
				
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
				//
				printf("Hare cambio de modo dentro del nodo\n");
				
				command[i].mode = controlMode;
				
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
		
		//
		std::cout << "Le mandare esto al motor 1: " << unsigned(param1_goal_state[0]) << unsigned(param1_goal_state[1]) << unsigned(param1_goal_state[2]) << unsigned(param1_goal_state[3]) << std::endl;
		//printf("Le mandare esto al motor 1: %d",param1_goal_state);
		
		param2_goal_state[0] = DXL_LOBYTE(DXL_LOWORD((int)command[1].value));
		param2_goal_state[1] = DXL_HIBYTE(DXL_LOWORD((int)command[1].value));
		param2_goal_state[2] = DXL_LOBYTE(DXL_HIWORD((int)command[1].value));
		param2_goal_state[3] = DXL_HIBYTE(DXL_HIWORD((int)command[1].value));
		
		//
		std::cout << "Le mandare esto al motor 2: " << unsigned(param2_goal_state[0]) << unsigned(param2_goal_state[1]) << unsigned(param2_goal_state[2]) << unsigned(param2_goal_state[3]) << std::endl;
		//printf("Le mandare esto al motor 2: %d",param2_goal_state);
		
		param3_goal_state[0] = DXL_LOBYTE(DXL_LOWORD((int)command[2].value));
		param3_goal_state[1] = DXL_HIBYTE(DXL_LOWORD((int)command[2].value));
		param3_goal_state[2] = DXL_LOBYTE(DXL_HIWORD((int)command[2].value));
		param3_goal_state[3] = DXL_HIBYTE(DXL_HIWORD((int)command[2].value));
		
		//
		std::cout << "Le mandare esto al motor 3: " << unsigned(param3_goal_state[0]) << unsigned(param3_goal_state[1]) << unsigned(param3_goal_state[2]) << unsigned(param3_goal_state[3]) << std::endl;
		//printf("Le mandare esto al motor 3: %d",param3_goal_state);
		
		param4_goal_state[0] = DXL_LOBYTE(DXL_LOWORD((int)command[3].value));
		param4_goal_state[1] = DXL_HIBYTE(DXL_LOWORD((int)command[3].value));
		param4_goal_state[2] = DXL_LOBYTE(DXL_HIWORD((int)command[3].value));
		param4_goal_state[3] = DXL_HIBYTE(DXL_HIWORD((int)command[3].value));
		
		//
		std::cout << "Le mandare esto al motor 4: " << unsigned(param4_goal_state[0]) << unsigned(param4_goal_state[1]) << unsigned(param4_goal_state[2]) << unsigned(param4_goal_state[3]) << std::endl;
		//printf("Le mandare esto al motor 4: %d",param4_goal_state);

		// Add parameter storage for Dynamixels goal
		paramStorageWrite(DXL1_ID, addressGoal[0], lenSize[0], param1_goal_state);
		if(exitParam){
			printf("Param write 1 (return 0) failed");
			exitParam = false;
		}
		
		paramStorageWrite(DXL2_ID, addressGoal[1], lenSize[1], param2_goal_state);
		if(exitParam){
			printf("Param write 2 (return 0) failed");
			exitParam = false;
		}
		
		paramStorageWrite(DXL3_ID, addressGoal[2], lenSize[2], param3_goal_state);
		if(exitParam){
			printf("Param write 3 (return 0) failed");
			exitParam = false;
		}
		
		paramStorageWrite(DXL4_ID, addressGoal[3], lenSize[3], param4_goal_state);
		if(exitParam){
			printf("Param write 4 (return 0) failed");
			exitParam = false;
		}

		// Bulkwrite goal joint states
		dxl_comm_result = commandPacket.txPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		
		//
		printf("Ya lo envie\n");

		// Clear bulkwrite parameter storage
		commandPacket.clearParam();
	}
	
	// Method to publish present values of the motors
	void publishJointState(){
		// Check present states
		// Obtains present values and prints them
		assignReadParam();
		
		if(exitParam){
			printf("Could not obtain present value for publisher (return 0)");
			exitParam = false;
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
			printf("Dynamixel#%d's torque has been enabled \n", id);
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
		
		enableTorque(id);
	}

	// Add parameter storage for all dynamixels
	void paramStorageRead(uint8_t id){
		// Add parameter storage for Dynamixel#id present position
		dxl_addparam_result = positionPacket.addParam(id, ADDR_PRESENT_POSITION, LEN_PV);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (position)", id);
			exitParam = true;
		}
		
		// Add parameter storage for Dynamixel#id present velocity
		dxl_addparam_result = velocityPacket.addParam(id, ADDR_PRESENT_VELOCITY, LEN_PV);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (velocity)", id);
			exitParam = true;
		}
		
		//Motors 3 and 4 (rotational) do not have torque control, there's no need to read the value
		//However, we could technically read present load in the motors
		//In said case, remove the if condition which skips id 3 and 4
		if(id != 3 && id != 4){
			// Add parameter storage for Dynamixel#id present torque
			dxl_addparam_result = torquePacket.addParam(id, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			if (dxl_addparam_result != true){
				fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (torque)", id);
				exitParam = true;
			}
		}
	}

	void readAvailable(dynamixel::GroupBulkRead &groupBulkRead, uint8_t id, int address, int len){
		dxl_getdata_result = groupBulkRead.isAvailable(id, address, len);
		if (dxl_getdata_result != true){
			fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", id);
			exitParam = true;
		}
	}

	void getPresentValue(uint8_t id1, uint8_t id2){
		// Bulkread present state
		dxl_comm_result = positionPacket.txRxPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (positionPacket.getError(id1, &dxl_error)){
			printf("[ID:%03d] %s\n", id1, packetHandler->getRxPacketError(dxl_error));
		}
		else if (positionPacket.getError(id2, &dxl_error)){
			printf("[ID:%03d] %s\n", id2, packetHandler->getRxPacketError(dxl_error));
		}
		
		readAvailable(positionPacket, id1, ADDR_PRESENT_POSITION, LEN_PV);
		if(exitParam){
			exitParam = false;
			//return;
		}
		
		readAvailable(positionPacket, id2, ADDR_PRESENT_POSITION, LEN_PV);
		if(exitParam){
			exitParam = false;
			//return;
		}
		
		// Store initial values
		stateArray[id1 - 1].id = std::to_string(id1);
		stateArray[id1 - 1].position = positionPacket.getData(id1, ADDR_PRESENT_POSITION, LEN_PV);
		stateArray[id2 - 1].id = std::to_string(id2);
		stateArray[id2 - 1].position = positionPacket.getData(id2, ADDR_PRESENT_POSITION, LEN_PV);
		
		dxl_comm_result = velocityPacket.txRxPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (velocityPacket.getError(id1, &dxl_error)){
			printf("[ID:%03d] %s\n", id1, packetHandler->getRxPacketError(dxl_error));
		}
		else if (velocityPacket.getError(id2, &dxl_error)){
			printf("[ID:%03d] %s\n", id2, packetHandler->getRxPacketError(dxl_error));
		}
		
		readAvailable(velocityPacket, id1, ADDR_PRESENT_VELOCITY, LEN_PV);
		if(exitParam){
			exitParam = false;
			//return;
		}
		
		readAvailable(velocityPacket, id2, ADDR_PRESENT_VELOCITY, LEN_PV);
		if(exitParam){
			exitParam = false;
			//return;
		}
		
		stateArray[id1 - 1].velocity = velocityPacket.getData(id1, ADDR_PRESENT_VELOCITY, LEN_PV);
		stateArray[id2 - 1].velocity = velocityPacket.getData(id2, ADDR_PRESENT_VELOCITY, LEN_PV);
		
		// Check for negative values
		if(stateArray[id1 - 1].velocity > (pow(2,31))){
			stateArray[id1 - 1].velocity = stateArray[id1 - 1].velocity - (pow(2,32)) - 1;
		}
		if(stateArray[id2 - 1].velocity > (pow(2,31))){
			stateArray[id2 - 1].velocity = stateArray[id2 - 1].velocity - (pow(2,32)) - 1;
		}
		
		if((id1 == DXL1_ID || id1 == DXL2_ID) && (id2 == DXL1_ID || id2 == DXL2_ID)){
			dxl_comm_result = torquePacket.txRxPacket();
			if (dxl_comm_result != COMM_SUCCESS){
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (torquePacket.getError(id1, &dxl_error)){
				printf("[ID:%03d] %s\n", id1, packetHandler->getRxPacketError(dxl_error));
			}
			else if (torquePacket.getError(id2, &dxl_error)){
				printf("[ID:%03d] %s\n", id2, packetHandler->getRxPacketError(dxl_error));
			}
			
			readAvailable(torquePacket, id1, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			if(exitParam){
				exitParam = false;
				//return;
			}
			
			readAvailable(torquePacket, id2, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			if(exitParam){
				exitParam = false;
				//return;
			}
			
			stateArray[id1 - 1].torque = torquePacket.getData(id1, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			stateArray[id2 - 1].torque = torquePacket.getData(id2, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			
			// Check for negative values
			if(stateArray[id1 - 1].torque > (pow(2,15))){
				stateArray[id1 - 1].torque = stateArray[id1 - 1].torque - (pow(2,16)) - 1;
			}
			if(stateArray[id2 - 1].torque > (pow(2,15))){
				stateArray[id2 - 1].torque = stateArray[id2 - 1].torque - (pow(2,16)) - 1;
			}
		}	
	}

	void paramStorageWrite(uint8_t id, int address, int len, uint8_t param_goal_position[4]){
		dxl_addparam_result = commandPacket.addParam(id, address, len, param_goal_position);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] commandPacket addparam failed", id);
			exitParam = true;
			//return;
		}
	}
	
	void assignReadParam(){
		// Check Spinning motors first
		// Add parameter storage for Dynamixel#1 present values
		paramStorageRead(DXL1_ID);
		if(exitParam){
			printf("\nParameter storage for DXL1 (return 0)");
			exitParam = false;
		}
		
		// Add parameter storage for Dynamixel#2 present values
		paramStorageRead(DXL2_ID);
		if(exitParam){
			printf("\nParameter storage for DXL2 (return 0)");
			exitParam = false;
		}
		
		// Get initial values
		getPresentValue(DXL1_ID, DXL2_ID);
		if(exitParam){
			printf("\nError in getting present values (return 0)");
			exitParam = false;
		}
		
		positionPacket.clearParam();
		velocityPacket.clearParam();
		torquePacket.clearParam();
		
		// Check rotation motors second
		// Add parameter storage for Dynamixel#3 present values
		paramStorageRead(DXL3_ID);
		if(exitParam){
			printf("\nParameter storage for DXL3 (return 0)");
			exitParam = false;
		}
		
		// Add parameter storage for Dynamixel#4 present values
		paramStorageRead(DXL4_ID);
		if(exitParam){
			printf("\nParameter storage for DXL4 (return 0)");
			exitParam = false;
		}
		
		// Get initial values
		getPresentValue(DXL3_ID, DXL4_ID);
		if(exitParam){
			printf("\nError in getting present values (return 0)");
			exitParam = false;
		}
		
		positionPacket.clearParam();
		velocityPacket.clearParam();
		
		//printMotorState();
	}
	
	// TODO Quitar los print en español
	void printMotorState(){
		printf("ID 1 debe estar en modo: %d", command[0].mode);
		printf("[ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \t Present Torque : %.2f \n", DXL1_ID, stateArray[0].position, stateArray[0].velocity, stateArray[0].torque);
		printf("ID 2 debe estar en modo: %d", command[1].mode);
		printf("[ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \t Present Torque : %.2f \n", DXL2_ID, stateArray[1].position, stateArray[1].velocity, stateArray[1].torque);
		printf("ID 3 debe estar en modo: %d", command[2].mode);
		printf("[ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \n", DXL3_ID, stateArray[2].position, stateArray[2].velocity);
		printf("ID 4 debe estar en modo: %d", command[3].mode);
		printf("[ID:%03d] \n Present Position : %.2f \t Present Velocity : %.2f \n", DXL4_ID, stateArray[3].position, stateArray[3].velocity);
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
