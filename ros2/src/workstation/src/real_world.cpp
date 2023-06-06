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

struct Point{
    float x;
    float y;
};
class real_world : public rclcpp::Node
{
    public :
        real_world(rclcpp::NodeOptions options) : Node("real_world", options)
        {
            //declare_parameter("frequency", 20);
            //get_parameter("frequency", frequency);
            //Create transform broadcaster
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            position_subscriber = this->create_subscription<control_input::msg::PositionCommand>(
                "position_cmd", 10, std::bind(&real_world::jointCommandFromPositionCmd, this, std::placeholders::_1));

            control_input_subscriber = this->create_subscription<control_input::msg::ControlInput>(
                "control_cmd", 10, std::bind(&real_world::jointCommandFromControlCmd, this, std::placeholders::_1));

            joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&real_world::motorStates, this, std::placeholders::_1));

            joint_command_publisher = this->create_publisher<sensor_msgs::msg::JointState>("motor_cmd", 10);
            state_vector_publisher=this->create_publisher<control_input::msg::StateVector>("state_vector", 10);

        }

    private :

        float a=0.08;//Base distance in meters
        float R=0.033; //Radius of the wheels in meters
        float v1=0, v2=0; //Tangent velocities of the wheels
        float dd1Cmd=0, dd2Cmd=0, phi1dCmd=0, phi2dCmd=0, d1Cmd=0, d2Cmd=0;

        //We initialize all variables of the state vector at 0
        float tt=0, x=0, y=0, phi1=0, phi2=0, phi1d=0, phi2d=0, beta1=0, beta2=0, Um=0, dd1=0, dd2=0, d1=0, d2=0, tt_dot=0, x_dot=0, y_dot=0;
        float frequency = 20; // HZ, used for calculating positions via integrals
        float period = 1/frequency; //Seconds
        
        Point ICRLocation;
        tf2::Quaternion rotation;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::JointState joint_cmd;
        geometry_msgs::msg::TransformStamped icr;
        sensor_msgs::msg::JointState joint_state;
        control_input::msg::StateVector robot_state;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        std::vector<std::string> names={"right_wheel_base_joint", "right_wheel_joint", "left_wheel_base_joint", "left_wheel_joint"};


        //Defining publishers
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher;
        rclcpp::Publisher<control_input::msg::StateVector>::SharedPtr state_vector_publisher;

        //Defining subscribers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
        rclcpp::Subscription<control_input::msg::PositionCommand>::SharedPtr position_subscriber;
        rclcpp::Subscription<control_input::msg::ControlInput>::SharedPtr control_input_subscriber;

    void motorStates(const sensor_msgs::msg::JointState::SharedPtr motor){
        auto start = motor->name.begin();
        auto d1_index = std::find(motor->name.begin(), motor->name.end(), "right_wheel_base_joint") - start;
        auto d2_index = std::find(motor->name.begin(), motor->name.end(), "left_wheel_base_joint") - start;

        d1 = motor->position[d1_index];
        d2 = limit_angle(motor->position[d2_index] - M_PI);
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
        dd1Cmd=limit_deltaSpeed(msg->delta1dot);
        dd2Cmd=limit_deltaSpeed(msg->delta2dot);

        // Change in position for d1 and d2
        d1Cmd = d1 + (dd1Cmd * period);
        //d2Cmd = d2 + (dd2Cmd * period);
        d2Cmd = -d1Cmd;

        phi1dCmd=limit_phiSpeed(2*cos(d2)*Um/R);
        phi2dCmd=limit_phiSpeed(2*cos(d1)*Um/R);

        joint_cmd.name.push_back(names[0]);
        joint_cmd.name.push_back(names[2]);
        joint_cmd.position.push_back(d1Cmd);
        //joint_cmd.position.push_back(0);
        joint_cmd.position.push_back(d2Cmd);
        //joint_cmd.position.push_back(0);
        publishJointCommand();

        joint_cmd.name.push_back(names[1]);
        joint_cmd.name.push_back(names[3]);
        //joint_cmd.velocity.push_back(dd1Cmd);
        joint_cmd.velocity.push_back(phi1dCmd);
        //joint_cmd.velocity.push_back(dd2Cmd);
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
