/*Real world node


################### BEGIN EXPLANATION ######################

    The objective of this node is to convert the control input command into joint commands which are then published in the motor_cmd topic

    It also obtains the joint speeds of the real robot and calculates its pose with respect to the odom frame
    Then it assembles a transform and publishes it to /tf to be able to see the robot's motion in rviz

    We subscribe to control_cmd or position_cmd depending on the value of the "mode" argument sent to this node.

    if mode="position", The node will subscribe to position_cmd and forward the value to the motor_cmd topic
    if mode="velocity" or "controller", the node will subscribe to control_cmd and generate the desired joint speeds to be sent to motor_cmd from a control input. 
     

---Subscriptions:
    /position_cmd
    /control_cmd
    /joint_states
    
---Publishers
    /joint_cmd
    /state_vector

    To convert joint states into tf2 transform:

    We subscribe to the topic joint_states via state_susbscriber 
    This subscriber will receive a JointState type message, which will include a list of the states of each of the four motors. 
    (position, speed, and torque)
    To calculate current pose, we obtain (solve from the rows corresponding to PHI in the S(q) matrix. 
    We calculate the current twist using the motor speeds and then integrate over time to obtain the current pose.
    

    ---To calculate the desired joint speeds---
    We obtain the control_cmd (or position_cmd in case of manual position mode)
    We multiply by UM the corresponding rows of S(q) in case of control_cmd
    Finally we publish the list of obtained values as a jointState message in the joint_cmd topic

#################### END EXPLANATION #####################
*/


#include <string>
#include <math.h>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_input/msg/state_vector.hpp>
#include <control_input/msg/motor_command.hpp>
#include <control_input/msg/control_input.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_input/msg/slider_command.hpp>
#include <control_input/msg/position_command.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <robot.h>

using namespace std::chrono_literals;

float limit_phiSpeed(float a){
    float max=(75.0/60.0)*(2*M_PI); //Limit phi rotation speeds to +-75rpm(The dynamixel XM430-W210 MAXIMUM speed)
    if( a >=  max) {a =max;}
    else if(a<= -max) {a=-max;} 
    return a;
}

float limit_deltaSpeed(float a){ //Limit delta rotation speeds to +-55rpm (The dynamixel MX28 MAXIMUM speed)
    float max=(55.0/60.0)*(2*M_PI);
    if( a >=  max) {a =max;}
    else if(a<= -max) {a=-max;} 
    return a;
}

double limit_angle(double a){
    while( a >  M_PI ) a -= 2*M_PI ;
    while( a <  -M_PI ) a += 2*M_PI ;
    return a ;
}
class real_world : public rclcpp::Node
{
    public :
        real_world(rclcpp::NodeOptions options) : Node("real_world", options)
        {
            declare_parameter("frequency", 20);
            frequency = this->get_parameter("frequency").as_int();
            period = 1.0/frequency; //Seconds
            //Create transform broadcaster

            position_subscriber = this->create_subscription<control_input::msg::PositionCommand>(
                "position_cmd", 10, std::bind(&real_world::jointCommandFromPositionCmd, this, std::placeholders::_1));

            control_input_subscriber = this->create_subscription<control_input::msg::ControlInput>(
                "control_cmd", 10, std::bind(&real_world::jointCommandFromControlCmd, this, std::placeholders::_1));

            joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&real_world::motorStates, this, std::placeholders::_1));

            joint_command_publisher = this->create_publisher<sensor_msgs::msg::JointState>("motor_cmd", 10);

            turtle4=Robot(0.0, 0, 0, 0.033, 0.33, 0.3, 0.16, 2);
        }

    private :
        Robot turtle4;
        float dd1Cmd=0, dd2Cmd=0, phi1dCmd=0, phi2dCmd=0, d1Cmd=0, d2Cmd=0, t1Cmd=0, t2Cmd=0;
        //We initialize all variables of the state vector at 0

        float beta1=0, beta2=0, Um=0;
        float cd1=0,sd1=0,cd2=0, sd2=0, ct=0, st=0,phi1d_prev=0, phi2d_prev=0;

        float frequency; // HZ, used for calculating positions via integrals
        float period;
        float l1 = 0.08; //Meters
        float zz= turtle4.mass*(pow(turtle4.chassis_width, 2)+ pow(turtle4.chassis_length, 2))/12;
        sensor_msgs::msg::JointState joint_cmd;
        std::vector<std::string> names={"right_wheel_base_joint", "right_wheel_joint", "left_wheel_base_joint", "left_wheel_joint"};


        //Defining publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher;
        //rclcpp::Publisher<control_input::msg::StateVector>::SharedPtr state_vector_publisher;
        
        //Defining subscribers
        //rclcpp::Subscription<control_input::msg::StateVector>::SharedPtr state_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
        rclcpp::Subscription<control_input::msg::PositionCommand>::SharedPtr position_subscriber;
        rclcpp::Subscription<control_input::msg::ControlInput>::SharedPtr control_input_subscriber;

    void motorStates(const sensor_msgs::msg::JointState::SharedPtr motor){
        auto start = motor->name.begin();
        auto d1_index = std::find(motor->name.begin(), motor->name.end(), turtle4.delta1.name) - start;
        auto d2_index = std::find(motor->name.begin(), motor->name.end(), turtle4.delta2.name) - start;
        auto phi1_index = std::find(motor->name.begin(), motor->name.end(), turtle4.phi1.name) - start;
        auto phi2_index = std::find(motor->name.begin(), motor->name.end(), turtle4.phi2.name) - start;

        turtle4.setMotorPositions(motor->position[phi1_index],motor->position[phi2_index], motor->position[d1_index], limit_angle(motor->position[d2_index] - M_PI));
        turtle4.setMotorVelocities(motor->velocity[phi1_index], motor->velocity[phi2_index], motor->velocity[d1_index], motor->velocity[d2_index]);

        //phi1d_prev=turtle4.phi1.velocity;
        //phi1d_prev=turtle4.phi2.velocity;
        //turtle4.phi1.acceleration=(phi1d_prev-turtle4.phi1.velocity)/period;
        //turtle4.phi2.acceleration=(phi2d_prev-turtle4.phi2.velocity)/period;


    }

    void jointCommandFromPositionCmd(const control_input::msg::PositionCommand::SharedPtr msg){
        for(uint i=0;i<names.size(); i++){
            joint_cmd.name.push_back(names[i]);
        }

        joint_cmd.position.push_back(msg->d1);
        joint_cmd.position.push_back(msg->phi1);
        joint_cmd.position.push_back(msg->d2);
        joint_cmd.position.push_back(msg->phi2); 
        publishJointCommand();  
    }

    void jointCommandFromControlCmd(const control_input::msg::ControlInput::SharedPtr msg){

        Um=msg->um;
        d1Cmd=limit_angle(msg->delta1);
        d2Cmd=limit_angle(msg->delta2);

        phi1dCmd=limit_phiSpeed(2*cos(d2Cmd)*Um/turtle4.wheel_radius);
        phi2dCmd=limit_phiSpeed(2*cos(d1Cmd)*Um/turtle4.wheel_radius);

        // cd1=cos(turtle4.delta1.position);
        // sd1=sin(turtle4.delta1.position);
        // cd2=cos(turtle4.delta2.position);
        // sd2=sin(turtle4.delta2.position);
        // ct=cos(turtle4.pose.theta);
        // st=sin(turtle4.pose.theta);
        // float aux1=0, aux2=0, aux3=0;

        // aux1 = turtle4.mass*(4*pow(turtle4.wheel_radius,2)*sd1*st*ct*turtle4.delta1.velocity -4*pow(turtle4.wheel_radius,2)*sd1*pow(st, 2)*cd1*turtle4.delta1.velocity*phi1dCmd + 4*pow(turtle4.wheel_radius,2)*sd1*cd1*pow(ct,2)*turtle4.delta1.velocity*phi1dCmd
        //                 +4*pow(turtle4.wheel_radius,2)*sd2*pow(st,2)*cd1*phi2dCmd*turtle4.twist.angular.z - 4*pow(turtle4.wheel_radius,2)*sd2*st*cd1*ct*turtle4.phi2.acceleration -4*pow(turtle4.wheel_radius,2)*sd2*cd1*pow(ct,2)*phi2dCmd*turtle4.twist.angular.z
        //                 +2*pow(turtle4.wheel_radius,2)*pow(st,2)*pow(cd1,2)*turtle4.phi1.acceleration - 4*pow(turtle4.wheel_radius,2)*st*cd1*cd2*ct*turtle4.delta2.velocity*phi2dCmd + 2*pow(turtle4.wheel_radius,2)*pow(cd1,2)*pow(ct,2)*turtle4.phi1.acceleration);

        // aux2 = (pow(turtle4.wheel_radius,2)*l1/(turtle4.wheel_distance))*(-2*pow(sd1,2)*ct*turtle4.delta1.velocity*phi1dCmd + sd1*sd2*ct*turtle4.delta1.velocity*phi2dCmd - 2*sd1*st*cd1*phi1dCmd*turtle4.twist.angular.z + 2*sd1*cd1*ct*turtle4.phi1.acceleration 
        //                             -pow(sd2,2)*st*turtle4.phi2.acceleration - pow(sd2,2)*ct*phi2dCmd*turtle4.twist.angular.z + sd2*st*cd1*phi2dCmd*turtle4.twist.angular.z - 2*sd2*st*cd2*turtle4.delta2.velocity*phi2dCmd
        //                             - sd2*cd1*ct*turtle4.phi2.acceleration + 2*pow(cd1,2)*ct*turtle4.delta1.velocity*phi1dCmd - cd1*cd2*ct*turtle4.delta2.velocity*phi2dCmd);
        
        // aux3 = (zz/pow(turtle4.wheel_distance, 2))*(pow(turtle4.wheel_radius,2)*pow(sd1,2)*turtle4.phi1.acceleration - pow(turtle4.wheel_radius,2)*sd1*sd2*turtle4.phi2.acceleration + 2*pow(turtle4.wheel_radius,2)*sd1*cd1*turtle4.delta1.velocity*phi1dCmd - pow(turtle4.wheel_radius,2)*sd1*cd2*turtle4.delta2.velocity*phi2dCmd - pow(turtle4.wheel_radius,2)*sd2*cd1*turtle4.delta1.velocity*phi2dCmd);

        // t1Cmd=(aux1+aux2+aux3)*0.5;

        joint_cmd.name.push_back(names[0]);
        joint_cmd.name.push_back(names[2]);
        joint_cmd.position.push_back(d1Cmd);
        joint_cmd.position.push_back(d2Cmd);
        publishJointCommand();

        joint_cmd.name.push_back(names[1]);
        joint_cmd.name.push_back(names[3]);
        joint_cmd.velocity.push_back(phi1dCmd);
        joint_cmd.velocity.push_back(phi2dCmd);
        publishJointCommand();

    }

    void publishJointCommand(){
        joint_cmd.header.stamp=this->now();
        joint_command_publisher->publish(joint_cmd);

        //We clear the command for next time
        joint_cmd.velocity.clear();
        joint_cmd.position.clear();
        joint_cmd.effort.clear();
        joint_cmd.name.clear();
    }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<real_world>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
