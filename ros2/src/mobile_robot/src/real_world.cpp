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
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

struct MotorState{
    std::string id;
    float position;
    float speed;
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

            state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                            "joint_states", 10, std::bind(&real_world::calculatePose, this, std::placeholders::_1));

            control_input_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                            "input_cmd", 10, std::bind(&real_world::calculateDesiredSpeeds, this, std::placeholders::_1));

            joint_command_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_cmd", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&real_world::publishJointCommand, this));
            
        }
        //We initialize all variables of the state vector at 0
        float tt, x, y, phi1, phi2, phi1d, phi2d, beta1, beta2, Um, dd1, dd2, d1, d2, tt_dot, x_dot, y_dot=0;
        

    private :
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        sensor_msgs::msg::JointState joint_cmd;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr control_input_subscriber;

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
    void calculateDesiredSpeeds(const std_msgs::msg::Float64MultiArray::SharedPtr inputCommand){
        Um = inputCommand->data[0];
        dd1 = inputCommand->data[1];
        dd2 = inputCommand->data[2];

        phi1d=2*cos(d2)*Um/R;
        phi2d=2*cos(d1)*Um/R;
        commandArray[0].id=1;
        commandArray[0].speed=phi1d;

        commandArray[1].id=2;
        commandArray[1].speed=phi2d;

        commandArray[2].id=3;
        commandArray[2].speed=dd1;

        commandArray[3].id=4;
        commandArray[4].speed=dd2;
        
    }
    void publishJointCommand(){
        
        joint_cmd.header.stamp=this->now();
        for(int i=0;i<4;i++){
            joint_cmd.name[i]=std::to_string(i);
            joint_cmd.velocity[i]=commandArray[i].speed;
            joint_cmd.position[i]=0;
            joint_cmd.effort[i]=0;
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