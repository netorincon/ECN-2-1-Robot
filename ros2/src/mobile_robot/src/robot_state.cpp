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
#include <chrono>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "DynamixelSDK.h"
#include <control_input/msg/state_vector.hpp>
#include <control_input/srv/reset_robot.hpp>

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

#define TRIES                           10  // Tries before giving up in communication error

struct MotorGoal{
	int id;
	int mode = 3;
	int value = 0;
};

struct MotorState{
	std::string id;
    float position = 0;
    float velocity = 0;
    float torque = 0;
};

struct Point{
    float x;
    float y;
};

class robot_state : public rclcpp::Node
{
	public :
        robot_state(rclcpp::NodeOptions options) : Node("robot_state", options){
            declare_parameter("frequency", 20);
            frequency = this->get_parameter("frequency").as_int();
            period = 1.0/frequency; //Seconds
            pid = getpid();
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

                    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
						
					// Subscriber for desired joint states		
					motor_command_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                                "motor_cmd",10,std::bind(&robot_state::goalJoints,this,std::placeholders::_1));
					
					// Publisher for present joint states
					joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
                    state_vector_publisher=this->create_publisher<control_input::msg::StateVector>("state_vector", 10);
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(int(period*1000)), std::bind(&robot_state::publishJointState,this));

                    // Service for reset
                    service = this->create_service<control_input::srv::ResetRobot>("robot_state/reset_robot_state",
                                std::bind(&robot_state::resetRobot,this,std::placeholders::_1,std::placeholders::_2));
				}
				else{
					printf("Failed to change the baudrate!\n");
                    exitNode();
					printf("Press CTRL + C terminate...\n");
				}
			}
			else{
				printf("Failed to open the port!\n");
                exitNode();
				printf("Press CTRL + C terminate...\n");
			}
        }
        
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

        geometry_msgs::msg::TransformStamped transform_stamped_;
        tf2::Quaternion rotation;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        control_input::msg::StateVector robot_state_vector;
        geometry_msgs::msg::TransformStamped icr;
        
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscriber;
        rclcpp::Publisher<control_input::msg::StateVector>::SharedPtr state_vector_publisher;
        rclcpp::Service<control_input::srv::ResetRobot>::SharedPtr service;

        int pid; // Get process ID
        MotorGoal command[4];		// Goal
        MotorState stateArray[4];	// Present

        float tt = 0, x = 0, y = 0, phi1 = 0, phi2 = 0, phi1d = 0, phi2d = 0, beta1 = 0, beta2 = 0, dd1 = 0, dd2 = 0, d1 = 0, d2 = 0, tt_dot = 0, x_dot = 0, y_dot = 0;
        float v1 = 0, v2 = 0;
        float frequency;                                // HZ, fréquence de publication des transformations sur le topic /tf
        float period;                                   // Seconds
        float a = 0.08;                                 // Base distance in meters
        float R = 0.033;                                // Radius of the wheels in meters
        Point ICRLocation;

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

    int torToPulse(float value){
        int tor = (((1.5862 * value) + 0.15) * 1000) / 2.69;
        if(abs(value) < 0.1){
            tor = 0;
        }
        return tor;
	}
	
	// Conversion methods for joint states (DXL -> radians)
	float pulseToPos(int value){
		return ((value % 4096) - 2048)*(M_PI / 2048); // Modulo to get absolute value (0 - 4095)
	}
	
	float pulseToVel(int value){
		return ((value % 1024) * 0.229 * 2 * M_PI) / 60; // DXL range 0 - 1023
	}

	float pulseToTor(int value){
        int tor = (((value * 2.69) / 1000) - 0.15) / 1.5862;
        if(value == 0){
            tor = 0;
        }
        return tor;
	}
	
	// Subscriber callback
	void goalJoints(const sensor_msgs::msg::JointState::SharedPtr cmd){
        int id, counter;
        bool validation; // Avoid assigning empty data to motors
        bool complete = true; // Avoid this reading if imcomplete
		
		for(int i = 0; i < (int)cmd->name.size(); i++){
			validation = true;
            counter = TRIES;
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
                complete = false;
			}
			
			if(validation){
				param_goal_state[0] = DXL_LOBYTE(DXL_LOWORD(command[id - 1].value));
				param_goal_state[1] = DXL_HIBYTE(DXL_LOWORD(command[id - 1].value));
				param_goal_state[2] = DXL_LOBYTE(DXL_HIWORD(command[id - 1].value));
				param_goal_state[3] = DXL_HIBYTE(DXL_HIWORD(command[id - 1].value));
				
				// Add parameter storage for Dynamixels goal
                do{
                    exitParam = false;
                    counter--;
                    paramStorageWrite(id, addressGoal[id - 1], lenSize[id - 1], param_goal_state);
                }while(exitParam && (counter > 0));

                // Last try and still error
                if((counter <= 0) && exitParam){
                    //THIS NODE SHOULD END
                    exitNode();
                }
			}
		}
		
        if(complete){
            // Bulkwrite goal joint states
            dxl_comm_result = commandPacket.txPacket();
            if (dxl_comm_result != COMM_SUCCESS){
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
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
            jointState.position.push_back(stateArray[i].position);
            jointState.velocity.push_back(stateArray[i].velocity);
            jointState.effort.push_back(stateArray[i].torque);
		}
		joint_state_publisher->publish(jointState);

        // Send odometry
        calculatePose();
	}

    //sensor_msgs::msg::JointState::SharedPtr
    // Service callback
    void resetRobot(control_input::srv::ResetRobot::Request::SharedPtr request, control_input::srv::ResetRobot::Response::SharedPtr response){
        int type = request->type;
        int counter;
        if(type == 0){
            // Reset odom and motors to 0
            for(int i = 0; i < 4; i++){
                counter = TRIES;
                changeMode(i+1,POSITION_CONTROL);

                command[i].mode = POSITION_CONTROL;
                addressGoal[i] = ADDR_GOAL_POSITION;
                lenSize[i] = LEN_PV;
                command[i].value = posToPulse(0);

                param_goal_state[0] = DXL_LOBYTE(DXL_LOWORD(command[i].value));
                param_goal_state[1] = DXL_HIBYTE(DXL_LOWORD(command[i].value));
                param_goal_state[2] = DXL_LOBYTE(DXL_HIWORD(command[i].value));
                param_goal_state[3] = DXL_HIBYTE(DXL_HIWORD(command[i].value));

                // Add parameter storage for Dynamixels goal
                do{
                    exitParam = false;
                    counter--;
                    paramStorageWrite(i+1, addressGoal[i], lenSize[i], param_goal_state);
                }while(exitParam && (counter > 0));

                // Last try and still error
                if((counter <= 0) && exitParam){
                    //THIS NODE SHOULD END
                    exitNode();
                }
            }

            // Bulkwrite goal joint states
            dxl_comm_result = commandPacket.txPacket();
            if (dxl_comm_result != COMM_SUCCESS){
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }

            // Clear bulkwrite parameter storage
            commandPacket.clearParam();

            assignReadParam();

            x = 0;
            y = 0;
            tt = 0;

            beta1 = stateArray[2].position;
            d1 = beta1;
            phi1 = stateArray[1].position;

            beta2 = stateArray[3].position;
            d2 = limitAngle(beta2 - M_PI);
            phi2 = stateArray[0].position;

            sendTransformBroadcaster();

            publishStateVectorOdom();

            response->status = 1;
        }
    }
	
	float limitAngle(float value){
        while(value >= M_PI){
			value = value - (2 * M_PI);
		}
        while(value <= (-M_PI)){
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

	void paramStorageWrite(uint8_t id, int address, int len, uint8_t param_goal_position[4]){
		dxl_addparam_result = commandPacket.addParam(id, address, len, param_goal_position);
		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] commandPacket addparam failed", id);
			exitParam = true;
		}
	}

    void readAvailable(dynamixel::GroupBulkRead &groupBulkRead, uint8_t id, int address, int len, bool &errorParam){
        dxl_getdata_result = groupBulkRead.isAvailable(id, address, len);
        if (dxl_getdata_result != true){
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", id);
            errorParam = true;
        }
    }

    void getPresentValue(uint8_t id1, uint8_t id2){
        bool positionError = false, velocityError = false, torqueError = false;
        stateArray[id1 - 1].id = MotorIds.at(id1);
        stateArray[id2 - 1].id = MotorIds.at(id2);

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

        readAvailable(positionPacket, id1, ADDR_PRESENT_POSITION, LEN_PV, positionError);
        readAvailable(positionPacket, id2, ADDR_PRESENT_POSITION, LEN_PV, positionError);
        if(!positionError){ // Saved in radians
            stateArray[id1 - 1].position = limitAngle(pulseToPos(positionPacket.getData(id1, ADDR_PRESENT_POSITION, LEN_PV)));
            stateArray[id2 - 1].position = limitAngle(pulseToPos(positionPacket.getData(id2, ADDR_PRESENT_POSITION, LEN_PV)));
            // Send beta2 instead of delta2
            if(id1 == DXL4_ID){
                stateArray[id1 - 1].position = limitAngle(stateArray[id1 - 1].position + M_PI);
            }
            else if(id2 == DXL4_ID){
                stateArray[id2 - 1].position = limitAngle(stateArray[id2 - 1].position + M_PI);
            }
        }

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

        readAvailable(velocityPacket, id1, ADDR_PRESENT_VELOCITY, LEN_PV, velocityError);
        readAvailable(velocityPacket, id2, ADDR_PRESENT_VELOCITY, LEN_PV, velocityError);
        if(!velocityError){ // Send in radians/second
            stateArray[id1 - 1].velocity = pulseToVel(velocityPacket.getData(id1, ADDR_PRESENT_VELOCITY, LEN_PV));
            stateArray[id2 - 1].velocity = pulseToVel(velocityPacket.getData(id2, ADDR_PRESENT_VELOCITY, LEN_PV));
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

            readAvailable(torquePacket, id1, ADDR_PRESENT_CURRENT, LEN_CURRENT, torqueError);
            readAvailable(torquePacket, id2, ADDR_PRESENT_CURRENT, LEN_CURRENT, torqueError);
            if(!torqueError){
                stateArray[id1 - 1].torque = pulseToTor(torquePacket.getData(id1, ADDR_PRESENT_CURRENT, LEN_CURRENT));
                stateArray[id2 - 1].torque = pulseToTor(torquePacket.getData(id2, ADDR_PRESENT_CURRENT, LEN_CURRENT));
            }
        }
        else{
            stateArray[id1 - 1].torque = 0;
            stateArray[id2 - 1].torque = 0;
        }

        if(positionError || velocityError || torqueError){
            exitParam = true;
        }
    }

    // Add parameter storage for dynamixel
    void paramStorageRead(uint8_t id){
        dxl_addparam_result = positionPacket.addParam(id, ADDR_PRESENT_POSITION, LEN_PV);
        if (dxl_addparam_result != true){
            fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (position)", id);
        }

        dxl_addparam_result = velocityPacket.addParam(id, ADDR_PRESENT_VELOCITY, LEN_PV);
        if (dxl_addparam_result != true){
            fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (velocity)", id);
        }

        //Motors 3 and 4 (rotational) do not have torque control, there's no need to read the value
        //However, we could technically read present load in the motors
        //In said case, remove the if condition which skips id 3 and 4
        if(id != DXL3_ID && id != DXL4_ID){
            dxl_addparam_result = torquePacket.addParam(id, ADDR_PRESENT_CURRENT, LEN_CURRENT);
            if (dxl_addparam_result != true){
                fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed (torque)", id);
            }
        }
    }
	
	void assignReadParam(){
        int counter = TRIES;

        do{
            exitParam = false;
            counter--;

            // Check Spinning motors first
            paramStorageRead(DXL1_ID);
            paramStorageRead(DXL2_ID);

            getPresentValue(DXL1_ID, DXL2_ID);

            positionPacket.clearParam();
            velocityPacket.clearParam();
            torquePacket.clearParam();

            // Check rotation motors second
            paramStorageRead(DXL3_ID);
            paramStorageRead(DXL4_ID);

            getPresentValue(DXL3_ID, DXL4_ID);

            positionPacket.clearParam();
            velocityPacket.clearParam();
        }while(exitParam && (counter > 0));
		
        // Last try and still error
        if((counter <= 0) && exitParam){
            //THIS NODE SHOULD END
            exitNode();
        }
	}
	
    // Odom
    void calculatePose(){
        //Get the current position and velocity of the motors
        beta1 = stateArray[2].position;
        d1 = beta1;
        phi1 = stateArray[1].position;

        beta2 = stateArray[3].position;
        d2 = limitAngle(beta2 - M_PI);
        phi2 = stateArray[0].position;

        dd1 = stateArray[2].velocity;
        phi1d = stateArray[1].velocity;
        dd2 = stateArray[3].velocity;
        phi2d = stateArray[0].velocity;

        v1 = phi1d * R; //Tangent speed of wheel one
        v2 = v1 * cos(d1) / cos(d2);

        //We calculate current robot speeds and orientation motor
        tt_dot = (1 / (2*a)) * (v1 * sin(d1) - v2 * sin(d2));

        tt += tt_dot * period;

        x_dot = v1 * cos(d1) * cos(tt) - v2 * sin(d2) * sin(tt);
        y_dot = v1 * cos(d1) * sin(tt) + v2 * sin(d2) * cos(tt);
        //We integrate the speeds over time (add each time we get a new value)
        x += x_dot * period;
        y += y_dot * period;
        tt = limitAngle(tt);

        sendTransformBroadcaster();

        //We publish the current state vector to be read by the controller
        publishStateVectorOdom();

        calculateICR();
        icr.header.frame_id = "base_link"; // Nom du repère fixe
        icr.child_frame_id = "icr"; // Nom du repère du robot
        icr.transform.translation.x = ICRLocation.x;
        icr.transform.translation.y = ICRLocation.y;
        icr.transform.translation.z = 0;
        icr.header.stamp = this->now();
        tf_broadcaster_->sendTransform(icr);
    }

    void sendTransformBroadcaster(){
        transform_stamped_.header.stamp = this->now();
        transform_stamped_.header.frame_id = "odom"; // Name of the fixed frame
        transform_stamped_.child_frame_id = "base_link"; // Name of the robot's frame
        transform_stamped_.transform.translation.x = x;
        transform_stamped_.transform.translation.y = y;
        transform_stamped_.transform.translation.z = 0;
        rotation.setRPY(0,0,tt);
        transform_stamped_.transform.rotation.x = rotation.getX();
        transform_stamped_.transform.rotation.y = rotation.getY();
        transform_stamped_.transform.rotation.z = rotation.getZ();
        transform_stamped_.transform.rotation.w = rotation.getW();
        tf_broadcaster_->sendTransform(transform_stamped_);
    }

    void publishStateVectorOdom(){
        robot_state_vector.x=x;
        robot_state_vector.y=y;
        robot_state_vector.theta=tt;
        robot_state_vector.delta1=d1;
        robot_state_vector.delta2=d2;
        robot_state_vector.delta3=0.0;
        robot_state_vector.phi1=phi1;
        robot_state_vector.phi2=phi2;
        state_vector_publisher->publish(robot_state_vector);
    }

    void calculateICR(){
        //We define alpha1 and alpha2 as temporary angles for ICR calculation
        float alpha1 = d1+M_PI/2;
        float alpha2 = d2+M_PI/2;
        alpha1 = limitAngle(alpha1);
        alpha2 = limitAngle(alpha2);

        float diff = alpha1 - alpha2;
        float diffAbs = abs(diff);
        if((diff>0) && (diffAbs<M_PI) && alpha1>=M_PI){
            alpha2+=M_PI;
        }
        else if((diff>0) && (diffAbs>=M_PI) && alpha1>=M_PI){
            alpha1-=M_PI;
        }
        else if((diff<0) && (diffAbs<M_PI) && (alpha2<M_PI)){
            alpha1+=M_PI;
            alpha2+=M_PI;
        }
        else if((diff<0) && (diffAbs<M_PI) && (alpha2>=M_PI)){
            alpha2-=M_PI;
        }
        else if((diff<0) && (diffAbs>=M_PI)){
            alpha1+=M_PI;
        }

        diff=alpha1-alpha2;
        //See PDF for detailed calculations
        if(sin(diff)!=0){
            ICRLocation.x=a+2*a*(sin(alpha2)*cos(alpha1)/sin(diff));
            ICRLocation.y=2*a*(sin(alpha1)*sin(alpha2)/sin(diff));
        }
    }

//	void printMotorState(){
//        printf("[ID:%03d] \n Present Position : %0.2f \t Present Velocity : %0.2f \t Present Torque : %0.2f \n", DXL1_ID, stateArray[0].position, stateArray[0].velocity, stateArray[0].torque);
//        printf("[ID:%03d] \n Present Position : %0.2f \t Present Velocity : %0.2f \t Present Torque : %0.2f \n", DXL2_ID, stateArray[1].position, stateArray[1].velocity, stateArray[1].torque);
//        printf("[ID:%03d] \n Present Position : %0.2f \t Present Velocity : %0.2f \n", DXL3_ID, stateArray[2].position, stateArray[2].velocity);
//        printf("[ID:%03d] \n Present Position : %0.2f \t Present Velocity : %0.2f \n", DXL4_ID, stateArray[3].position, stateArray[3].velocity);
//	}

    void exitNode(){
        kill(pid,SIGINT);
    }
};

int main(int argc, char** argv)
{	
	rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_state>(rclcpp::NodeOptions{});
	rclcpp::spin(node);
	
	node->disableTorque(DXL1_ID);
	node->disableTorque(DXL2_ID);
	node->disableTorque(DXL3_ID);
	node->disableTorque(DXL4_ID);
	node->portHandler->closePort();
	
	rclcpp::shutdown();
	return 0;
}
