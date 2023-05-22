#include <algorithm>
#include <string>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


#include <chrono>

struct MotorState{
    std::string id;
    float position;
    float speed;
};

class transform_broadcaster : public rclcpp::Node
{
    public :
        transform_broadcaster(rclcpp::NodeOptions options) : Node("transform_broadcaster", options)
        {


            //init transform broadcaster
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);


            /*
            We subscribe to two topics:

            cmd_susbscriber receives the input vector which includes [um, beta1dot and beta2dot]
            state_subscriber has the current state of the robot's motors (position, velocity and possibly torque)
            
            Each subscriber is tied to its own callback function (make_transforms and getState respectively)
            */
            cmd_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                            "cmd_robot", 10, std::bind(&transform_broadcaster::make_transforms, this, std::placeholders::_1));

            state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                            "motor_state", 10, std::bind(&transform_broadcaster::getState, this, std::placeholders::_1));
            
        }
        float tt = 0, x = 0, y = 0;
        MotorState stateArray[4];
        float a=0.05;

    private :
        geometry_msgs::msg::TransformStamped transform_stamped_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


void make_transforms(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {

        /*The goal is to estimate 'qdot' from the received input vector 'um'
        a transform is then assembled and published.

        To assemble the transform, we calculate the twist of the robot and then integrate it over time 
        to obtain the current position of the robot in the world frame. 

        */

        float Um = msg->data[0];
        float bt1 = msg->data[1];
        float bt2 = msg->data[2];

        int d1=stateArray[3].position;
        int d2=stateArray[2].position-M_PI;

        float tt_dot=Um*sin(d1-d2)/a; //sin(d1-d2)/a
        float x_dot=(2*cos(d1)*cos(d2)*cos(tt) - sin(d1+d2)*sin(tt))*Um;
        float y_dot=(2*cos(d1)*cos(d2)*sin(tt) + sin(d1+d2)*cos(tt))*Um;

        float frequency = 20; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;

        tt += tt_dot*period;
        x += x_dot*period;
        y += y_dot*period;


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
    }


    void getState(const sensor_msgs::msg::JointState::SharedPtr jointState){
        for(int i=0;i<4;i++){
            stateArray[i].id=jointState->name[i];
            stateArray[i].position=jointState->position[i];
            stateArray[i].speed=jointState->velocity[i];
        }
    }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<transform_broadcaster>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}