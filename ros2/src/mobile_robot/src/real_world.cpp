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

            // state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            //                 "joint_states", 10, std::bind(&real_world::calculatePose, this, std::placeholders::_1));

            control_input_subscriber = this->create_subscription<control_input::msg::ControlInput>(
                            "input_cmd", 10, std::bind(&real_world::jointCommandFromController, this, std::placeholders::_1));

            slider_subscriber = this->create_subscription<control_input::msg::SliderCommand>(
                            "slider_cmd", 10, std::bind(&real_world::jointCommandFromSliders, this, std::placeholders::_1));

            joint_command_publisher = this->create_publisher<control_input::msg::MotorCommand>("motor_cmd", 10);
            timer_ = this->create_wall_timer(50ms, std::bind(&real_world::publishJointCommand, this));
            
        }
        //We initialize all variables of the state vector at 0
        float tt, x, y, phi1, phi2, phi1d, phi2d, beta1, beta2, Um, dd1, dd2, d1, d2, tt_dot, x_dot, y_dot=0;
        

    private :
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        control_input::msg::MotorCommand joint_cmd;

        rclcpp::Publisher<control_input::msg::MotorCommand>::SharedPtr joint_command_publisher;
        // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;
        rclcpp::Subscription<control_input::msg::SliderCommand>::SharedPtr slider_subscriber;
        rclcpp::Subscription<control_input::msg::ControlInput>::SharedPtr control_input_subscriber;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        float frequency = 20; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;
        float a=0.05;//Base distance in meters
        float R=0.06; //Radius of the wheels in meters

        MotorState stateArray[4];
        MotorState commandArray[4];

    void calculatePose(const sensor_msgs::msg::JointState::SharedPtr jointState){

        //Save the values locally for easy access
        for(int i=0;i<4;i++){
            stateArray[i].id=jointState->name[i];
            stateArray[i].position=jointState->position[i];
            stateArray[i].speed=jointState->velocity[i];
        }
               
        //Get the current speed of the spin actuators
        phi1d=stateArray[2].speed; 
        phi2d=stateArray[3].speed;

        dd1=stateArray[0].speed;
        dd2=stateArray[1].speed;

        d1+=dd1*period; //delta 1
        d2+=dd2*period; //delta 2
        
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
        return;
    }
    void jointCommandFromSliders(const control_input::msg::SliderCommand::SharedPtr msg){
        if(msg->mode=="velocity"){
            Um = msg->um;
            dd1=msg->cmd.velocity[0];
            dd2=msg->cmd.velocity[1];

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
            commandArray[4].speed=dd2;   

            for(int i=0;i<4;i++){
                commandArray[i].position=0;
                commandArray[i].torque=0;
            }
            joint_cmd.mode={"velocity", "velocity", "velocity", "velocity"};
        }
        else if(msg->mode=="position"){
            Um=0;
            d1=msg->cmd.position[0];
            d2=msg->cmd.position[1];

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
            commandArray[4].position=d2;   

            for(int i=0;i<4;i++){
                commandArray[i].speed=0;
                commandArray[i].torque=0;
            }
            joint_cmd.mode={"position", "position", "position", "position"};
        }
    }

        void jointCommandFromController(const control_input::msg::ControlInput::SharedPtr msg){
            Um = msg->um;
            dd1=msg->beta1dot;
            dd2=msg->beta2dot;

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
            commandArray[4].speed=dd2;   

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