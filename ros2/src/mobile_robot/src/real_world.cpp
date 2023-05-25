/*Real world node

The objective of this node is to convert the control input command into joint commands which are then published in the joint_cmd topic

It also obtains the joint speeds of the real robot and calculates its position in the real world
Then it assembles a transform and published it to /tf2 to be able to see the robot's motion in rviz

---Subscriptions:
    /input_cmd
        The command input should include [um, deltaDot1 and deltaDot2]
    /joint_states topic
    
---Publishers
    /joint_cmd
*/

#include <algorithm>
#include <string>
#include <math.h>
#include <cstring>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_input/msg/slider_command.hpp>
#include <control_input/msg/motor_command.hpp>
#include <control_input/msg/control_input.hpp>
#include <control_input/msg/position_command.hpp>
#include <control_input/msg/state_vector.hpp>

using namespace std::chrono_literals;


//Structure for easy arranging of motor positions and speeds
struct MotorState{
    std::string id;
    float position;
    float speed;
    float torque;
};

class real_world : public rclcpp::Node
{
    public :
        real_world(rclcpp::NodeOptions options) : Node("real_world", options)
        {
            declare_parameter("mode", "velocity");
            get_parameter("mode", mode);
            //Create transform broadcaster
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            /*
            ---To convert joint states into tf2 transform:---
            We subscribe to the topic joint_states via state_susbscriber 
            This subscriber will receive a JointState type message, which will include a list of the states of each of the four motors. 
            (position, speed, and torque)
            To calculate current pose, we obtain (solve for) 'um' from the rows corresponding to PHI in the S(q) matrix. 
            Then we calculate the current twist using this um.
            Finally the values are integrated over time to obtain the current state vector.
            

            ---To calculate the desired joint speeds---
            We obtain the control input command
            We multiply by the corresponding rows of S(q)
            Finally we publish the list of obtained values as a jointState message in the joint_cmd topic

            */

            state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                             "motor_states", 10, std::bind(&real_world::calculatePose, this, std::placeholders::_1));

            if(mode=="position"){
                position_subscriber = this->create_subscription<control_input::msg::PositionCommand>(
                            "position_cmd", 10, std::bind(&real_world::jointCommandFromPositionCmd, this, std::placeholders::_1));

            }
            if(mode=="velocity" || mode=="controller"){
            control_input_subscriber = this->create_subscription<control_input::msg::ControlInput>(
                            "control_cmd", 10, std::bind(&real_world::jointCommandFromControlCmd, this, std::placeholders::_1));
            }
            

            joint_command_publisher = this->create_publisher<control_input::msg::MotorCommand>("motor_cmd", 10);
            state_vector_publisher=this->create_publisher<control_input::msg::StateVector>("state_vector", 10);
            joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

            timer_ = this->create_wall_timer(20ms, std::bind(&real_world::publishJointCommand, this));
            
        }

        

    private :
        //We initialize all variables of the state vector at 0
        float tt, x, y, phi1, phi2, phi1d, phi2d, beta1, beta2, Um, dd1, dd2, d1, d2, tt_dot, x_dot, y_dot=0;
        float frequency = 50; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;
        float a=0.05;//Base distance in meters
        float R=0.033; //Radius of the wheels in meters
        std::string mode;
        MotorState stateArray[4];
        MotorState commandArray[4];
        sensor_msgs::msg::JointState joint_state;
        control_input::msg::StateVector robot_state;

        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        control_input::msg::MotorCommand joint_cmd;
        rclcpp::Publisher<control_input::msg::MotorCommand>::SharedPtr joint_command_publisher;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;
        rclcpp::Subscription<control_input::msg::PositionCommand>::SharedPtr position_subscriber;
        rclcpp::Subscription<control_input::msg::ControlInput>::SharedPtr control_input_subscriber;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
        rclcpp::Publisher<control_input::msg::StateVector>::SharedPtr state_vector_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    void calculatePose(const sensor_msgs::msg::JointState::SharedPtr jointState){

        //Save the values locally for easy access
        for(int i=0;i<4;i++){
            stateArray[i].id=jointState->name[i];
            stateArray[i].position=jointState->position[i];
            stateArray[i].speed=jointState->velocity[i];
            stateArray[i].torque=jointState->effort[i];
        }
               
        //Get the current speed of the spin actuators
        phi1d=stateArray[2].speed; 
        phi2d=stateArray[3].speed;

        dd1=stateArray[0].speed;
        dd2=stateArray[1].speed;

        d1+=dd1*period; //delta 1
        d2+=dd2*period; //delta 2

        beta1=d1;
        beta2=d2+M_PI;
        phi1+=phi1d*period;
        phi2+=phi2d*period;

        //Now we solve for um using the equation for phi1 or phi2 from S(q) matrix
        float U=phi1d*R/(2*cos(d2));

        //We calculate current robot speeds and orientation motor 
        tt_dot=Um*sin(d1-d2)/a; //sin(d1-d2)/a
        x_dot=(2*cos(d1)*cos(d2)*cos(tt) - sin(d1+d2)*sin(tt))*U;
        y_dot=(2*cos(d1)*cos(d2)*sin(tt) + sin(d1+d2)*cos(tt))*U;

        //We integrate the speeds over time (add each time we get a new value)
        x+=x_dot*period;
        y+=y_dot*period;
        tt+=tt_dot*period;
        
        transform_stamped_.header.stamp = this->now();
        transform_stamped_.header.frame_id = "world"; // Nom du repère fixe
        transform_stamped_.child_frame_id = "chassis"; // Nom du repère du robot
        transform_stamped_.transform.translation.x = x;
        transform_stamped_.transform.translation.y = y;
        transform_stamped_.transform.translation.z = 0;
        transform_stamped_.transform.rotation.x += 0;
        transform_stamped_.transform.rotation.y += 0;
        transform_stamped_.transform.rotation.z += tt;
        transform_stamped_.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(transform_stamped_);

        //We publish the current state vector to be read by the controller
        robot_state.x=x;
        robot_state.y=y;
        robot_state.theta=tt;
        robot_state.delta1=d1;
        robot_state.delta2=d2;
        robot_state.phi1=phi1;
        robot_state.phi2=phi2;
        state_vector_publisher->publish(robot_state);

        return;
    }
    void jointCommandFromPositionCmd(const control_input::msg::PositionCommand::SharedPtr msg){
            Um=0;
            d1=msg->d1;
            d2=msg->d2;

            phi1d=0;
            phi2d=0;

            //Arrange the speeds in an array for faster assignment during publishJointCommand 
            commandArray[0].id=1;
            commandArray[0].position=0;

            commandArray[1].id=2;
            commandArray[1].position=0;

            commandArray[2].id=3;
            commandArray[2].position=d1;

            commandArray[3].id=4;
            commandArray[3].position=d2;   

            for(int i=0;i<4;i++){
                commandArray[i].speed=0;
                commandArray[i].torque=0;
            }
            joint_cmd.mode={"position", "position", "position", "position"};
    }

        void jointCommandFromControlCmd(const control_input::msg::ControlInput::SharedPtr msg){
            Um=msg->um;
            dd1=msg->delta1dot;
            dd2=msg->delta2dot;

            phi1d=2*cos(d2)*Um/R;
            phi2d=2*cos(d1)*Um/R;

            //Arrange the speeds in an array for faster assignment during publishJointCommand 
            commandArray[0].id=1;
            commandArray[0].speed=phi1d;

            commandArray[1].id=2;
            commandArray[1].speed=phi2d;

            commandArray[2].id=3;
            commandArray[2].speed=dd1;

            commandArray[3].id=4;
            commandArray[3].speed=dd2;   

            for(int i=0;i<4;i++){
                commandArray[i].position=0;
                commandArray[i].torque=0;
            }
            joint_cmd.mode={"velocity", "velocity", "velocity", "velocity"};

    }
    void publishJointCommand(){
        
        joint_cmd.cmd.header.stamp=this->now();
        joint_cmd.cmd.name.clear();
        joint_cmd.cmd.velocity.clear();
        joint_cmd.cmd.position.clear();
        joint_cmd.cmd.effort.clear();

        for(int i=0;i<4;i++){
            //TODO Change the name parameter to the robot joint name instead of using just a number
            joint_cmd.cmd.name.push_back(std::to_string(i+1));
            joint_cmd.cmd.velocity.push_back(commandArray[i].speed);
            joint_cmd.cmd.position.push_back(commandArray[i].position);
            joint_cmd.cmd.effort.push_back(commandArray[i].torque);
        }
        std::vector<std::string> names={"left_wheel_base_joint", "right_wheel_base_joint", "left_wheel_joint", "right_wheel_joint"};
        std::vector<double> posValues={beta2, beta1, phi2, phi1};
        std::vector<double> velValues={dd2, dd1, phi2d, phi1d};
        joint_state.name=names;
        joint_state.position = posValues;
        joint_state.header.stamp=this->now();
        joint_publisher->publish(joint_state);
        joint_command_publisher->publish(joint_cmd);
    }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<real_world>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}