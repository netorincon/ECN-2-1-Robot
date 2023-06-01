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
#include <map>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include "DynamixelSDK.h"

// Control table address
// EEPROM Area
// Data in EEPROM area can only be written when torque value (ADDR_TORQUE_ENABLE) is 0
#define ADDR_OPERATING_MODE				11
// RAM Area
// Rotation motors DO NOT have torque control
#define ADDR_TORQUE_ENABLE				64
#define ADDR_GOAL_CURRENT				102	// Does NOT exist in Rot motors
#define ADDR_GOAL_VELOCITY				104
#define ADDR_GOAL_POSITION				116
#define ADDR_PRESENT_CURRENT			126	// Represents "Present Load" in Rot motors
#define ADDR_PRESENT_VELOCITY			128
#define ADDR_PRESENT_POSITION			132

// Data Byte Length
#define LEN_CURRENT						2	// Torque
#define LEN_PV							4	// Position and Velocity

// Protocol version
#define PROTOCOL_VERSION                2.0

// Default setting
// Spinning motors
#define DXL1_ID                         1
#define DXL2_ID                         2
// Rotation motors
#define DXL3_ID                         3
#define DXL4_ID                         4

#define BAUDRATE						1000000
#define DEVICENAME						"/dev/ttyACM0"

#define TORQUE_ENABLE                   1	// Value for enabling the torque
#define TORQUE_DISABLE                  0	// Value for disabling the torque

#define POSITION_CONTROL				3
#define VELOCITY_CONTROL				1
#define TORQUE_CONTROL					0

struct MotorGoal{
	int id;
	int mode = 3;
	int value = 0;
};

struct MotorState{
	std::string id;
	int position = 0;
	int velocity = 0;
	int torque = 0;
};

class motor_state : public rclcpp::Node
{
	public :
		motor_state() : Node("motor_state"){
			
			// Open port
			if (portHandler->openPort()){
				printf("Succeeded to open the port!\n");
				
				// Set port baudrate
				if (portHandler->setBaudRate(BAUDRATE)){
					printf("Succeeded to change the baudrate!\n");
					
					// Enable Torque
					enableTorque(DXL1_ID);
					enableTorque(DXL2_ID);
					enableTorque(DXL3_ID);
					enableTorque(DXL4_ID);
					
					assignReadParam();
						
					// Initialize in mode 3 (Position Control) with current values
					for(int i = 0; i < 4; i++){
						command[i].id = i+1;
						command[i].mode = POSITION_CONTROL;
						command[i].value = stateArray[i].position;
						
						changeMode(command[i].id, command[i].mode);
					}
						
					// Subscriber for desired joint states		
					motor_command_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
								"motor_cmd",10,std::bind(&motor_state::goalJoints,this,std::placeholders::_1));
					
					// Publisher for present joint states
					joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
					timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&motor_state::publishJointState,this));
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
        
        MotorGoal command[4];		// Goal
        MotorState stateArray[4];	// Present
        
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
		}

    private :
		rclcpp::TimerBase::SharedPtr timer_;
		sensor_msgs::msg::JointState jointState;
        
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscriber;

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
		int lenSize[4] = {LEN_PV,LEN_PV,LEN_PV,LEN_PV};

		uint8_t dxl_error = 0;                          // Dynamixel error
		uint8_t param_goal_state[4];					// Goal state
		
		std::map<std::string, int> MotorNames{{"left_wheel_joint",1},{"right_wheel_joint",2},{"right_wheel_base_joint",3},{"left_wheel_base_joint",4}};
		std::map<int, std::string> MotorIds{{1,"left_wheel_joint"},{2,"right_wheel_joint"},{3,"right_wheel_base_joint"},{4,"left_wheel_base_joint"}};
		
	// Conversion methods for command values (radians -> DXL)
	int posToPulse(float value){
		int pos = 2048 + ((value * 2048) / M_PI);
		if(pos < 0 || pos > 4095){
			pos = 0;
		}
		return pos;
	}
	
	int velToPulse(float value){
		return ((value * 60) / (2 * M_PI)) / 0.229; // Increments are of 0.229 rpm
	}
	
	// TODO
	int torToPulse(float value){
		return value;
	}
	
	// Conversion methods for joint states (DXL -> radians)
	float pulseToPos(int value){
		return ((value % 4096) - 2048)*(M_PI / 2048); // Modulo to get absolute value (0 - 4095)
	}
	
	float pulseToVel(int value){
		return ((value % 1024) * 0.229 * 2 * M_PI) / 60; // DXL range 0 - 1023
	}
	
	// TODO
	float pulseToTor(int value){
		return value;
	}
	
	// Subscriber callback
	void goalJoints(const sensor_msgs::msg::JointState::SharedPtr cmd){
		int id;
		bool validation; // Avoid sending empty data to motors
		
		for(int i = 0; i < (int)cmd->name.size(); i++){
			validation = true;
			id = MotorNames.at(cmd->name[i]);
			command[id - 1].id = id;
			if(cmd->position.size() != 0){
				if(command[id - 1].mode != POSITION_CONTROL){
					changeMode(id, POSITION_CONTROL);
					command[id - 1].mode = POSITION_CONTROL;
					addressGoal[id - 1] = ADDR_GOAL_POSITION;
					lenSize[id - 1] = LEN_PV;
				}
				command[id - 1].value = posToPulse(cmd->position[i]);
			}
			else if(cmd->velocity.size() != 0){
				if(command[id - 1].mode != VELOCITY_CONTROL){
					changeMode(id, VELOCITY_CONTROL);
					command[id - 1].mode = VELOCITY_CONTROL;
					addressGoal[id - 1] = ADDR_GOAL_VELOCITY;
					lenSize[id - 1] = LEN_PV;
				}
				command[id - 1].value = velToPulse(cmd->velocity[i]);
			}
			else if((cmd->effort.size() != 0) && (id != DXL3_ID) && (id != DXL4_ID)){
				if(command[id - 1].mode != TORQUE_CONTROL){
					changeMode(id, TORQUE_CONTROL);
					command[id - 1].mode = TORQUE_CONTROL;
					addressGoal[id - 1] = ADDR_GOAL_CURRENT;
					lenSize[id - 1] = LEN_CURRENT;
				}
				command[id - 1].value = torToPulse(cmd->effort[i]);
			}
			else{
				printf("Motor command topic could not be read for motor ID : %d\n",id);
				validation = false;
			}
			
			if(validation){
				param_goal_state[0] = DXL_LOBYTE(DXL_LOWORD(command[id - 1].value));
				param_goal_state[1] = DXL_HIBYTE(DXL_LOWORD(command[id - 1].value));
				param_goal_state[2] = DXL_LOBYTE(DXL_HIWORD(command[id - 1].value));
				param_goal_state[3] = DXL_HIBYTE(DXL_HIWORD(command[id - 1].value));
				
				// Add parameter storage for Dynamixels goal
				paramStorageWrite(id, addressGoal[id - 1], lenSize[id - 1], param_goal_state);
				if(exitParam){
					exitParam = false;
				}
			}
		}
		
		// Bulkwrite goal joint states
		dxl_comm_result = commandPacket.txPacket();
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		
		// Clear bulkwrite parameter storage
		commandPacket.clearParam();
	}
	
	// Publisher callback
	void publishJointState(){
		// Check present states
		assignReadParam();
		
		jointState.header.stamp = this->now();
		jointState.name.clear();
		jointState.position.clear();
		jointState.velocity.clear();
		jointState.effort.clear();
		
		for(int i=0;i<4;i++){
			jointState.name.push_back(stateArray[i].id);
			if(stateArray[i].id == MotorIds.at(DXL4_ID)){ // Send beta 2 instead of delta 2
				jointState.position.push_back(limitAngle(pulseToPos(stateArray[i].position) + M_PI));
			}
			else{
				jointState.position.push_back(limitAngle(pulseToPos(stateArray[i].position)));
			}
			jointState.velocity.push_back(pulseToVel(stateArray[i].velocity));
			if(MotorNames.at(stateArray[i].id) != DXL3_ID && MotorNames.at(stateArray[i].id) != DXL4_ID){
				jointState.effort.push_back(pulseToTor(stateArray[i].torque));
			}
		}
		joint_state_publisher->publish(jointState);
	}
	
	float limitAngle(float value){
		while(value > M_PI){
			value = value - (2 * M_PI);
		}
		while(value < (-M_PI)){
			value = value + (2 * M_PI);
		}
		return value;
	}
	
	//ENABLE TORQUE
	void enableTorque(uint8_t id){
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS){
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
		else if (dxl_error != 0){
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
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

	// Add parameter storage for dynamixel
	void paramStorageRead(uint8_t id){
		dxl_addparam_result = positionPacket.addParam(id, ADDR_PRESENT_POSITION, LEN_PV);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (position)", id);
			exitParam = true;
			return;
		}
		
		dxl_addparam_result = velocityPacket.addParam(id, ADDR_PRESENT_VELOCITY, LEN_PV);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (velocity)", id);
			exitParam = true;
			return;
		}
		
		//Motors 3 and 4 (rotational) do not have torque control, there's no need to read the value
		//However, we could technically read present load in the motors
		//In said case, remove the if condition which skips id 3 and 4
		if(id != DXL3_ID && id != DXL4_ID){
			dxl_addparam_result = torquePacket.addParam(id, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			if (dxl_addparam_result != true){
				fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (torque)", id);
				exitParam = true;
				return;
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
			return;
		}
		
		readAvailable(positionPacket, id2, ADDR_PRESENT_POSITION, LEN_PV);
		if(exitParam){
			exitParam = false;
			return;
		}
		
		// Store initial values
		stateArray[id1 - 1].id = MotorIds.at(id1);
		stateArray[id1 - 1].position = positionPacket.getData(id1, ADDR_PRESENT_POSITION, LEN_PV);
		stateArray[id2 - 1].id = MotorIds.at(id2);
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
			return;
		}
		
		readAvailable(velocityPacket, id2, ADDR_PRESENT_VELOCITY, LEN_PV);
		if(exitParam){
			exitParam = false;
			return;
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
				return;
			}
			
			readAvailable(torquePacket, id2, ADDR_PRESENT_CURRENT, LEN_CURRENT);
			if(exitParam){
				exitParam = false;
				return;
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
		}
	}
	
	void assignReadParam(){
		// Check Spinning motors first
		paramStorageRead(DXL1_ID);
		if(exitParam){
			exitParam = false;
			return;
		}
		
		paramStorageRead(DXL2_ID);
		if(exitParam){
			exitParam = false;
			return;
		}
		
		getPresentValue(DXL1_ID, DXL2_ID);
		if(exitParam){
			exitParam = false;
		}
		
		positionPacket.clearParam();
		velocityPacket.clearParam();
		torquePacket.clearParam();
		
		// Check rotation motors second
		paramStorageRead(DXL3_ID);
		if(exitParam){
			exitParam = false;
			return;
		}
		
		// Add parameter storage for Dynamixel#4 present values
		paramStorageRead(DXL4_ID);
		if(exitParam){
			exitParam = false;
			return;
		}
		
		// Get initial values
		getPresentValue(DXL3_ID, DXL4_ID);
		if(exitParam){
			exitParam = false;
		}
		
		positionPacket.clearParam();
		velocityPacket.clearParam();
		
		//printMotorState();
	}
	
	void printMotorState(){
		printf("[ID:%03d] \n Present Position : %d \t Present Velocity : %d \t Present Torque : %d \n", DXL1_ID, stateArray[0].position, stateArray[0].velocity, stateArray[0].torque);
		printf("[ID:%03d] \n Present Position : %d \t Present Velocity : %d \t Present Torque : %d \n", DXL2_ID, stateArray[1].position, stateArray[1].velocity, stateArray[1].torque);
		printf("[ID:%03d] \n Present Position : %d \t Present Velocity : %d \n", DXL3_ID, stateArray[2].position, stateArray[2].velocity);
		printf("[ID:%03d] \n Present Position : %d \t Present Velocity : %d \n", DXL4_ID, stateArray[3].position, stateArray[3].velocity);
	}
};

int main(int argc, char** argv)
{	
	rclcpp::init(argc, argv);
	auto node = std::make_shared<motor_state>();
	rclcpp::spin(node);
	
	node->disableTorque(DXL1_ID);
	node->disableTorque(DXL2_ID);
	node->disableTorque(DXL3_ID);
	node->disableTorque(DXL4_ID);
	node->portHandler->closePort();
	
	rclcpp::shutdown();
	return 0;
}
