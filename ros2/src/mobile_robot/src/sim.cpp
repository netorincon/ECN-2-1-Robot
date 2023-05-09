/*Simulation node

The objective of this node is to be able to simulate the robot's motion 
only via software before the controller is deployed to the real robot. 

This node subscribes to the command_input topic.
The command input should include [um, deltaDot1 and deltaDot2]

*/

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

            //Create transform broadcaster
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            /*
            We subscribe to the topic cmd_robot via cmd_susbscriber 
            It receives the input vector which includes [um, beta1dot and beta2dot].
            qDot (derivative of the configuration vector) is estimated using the S matrix obtained in the Kinematic model calculations
            times the um input. 

            Then the values are integrated over time to obtain the current state vector.
            
            */
            cmd_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                            "cmd_robot", 10, std::bind(&transform_broadcaster::calculatePose, this, std::placeholders::_1));
            
        }
        //We initialize all variables of the state vector at 0
        float tt, x, y, phi1, phi2, beta1, beta2, Um, dd1, dd2, d1, d2, tt_dot, x_dot, y_dot=0;

    private :
        geometry_msgs::msg::TransformStamped transform_stamped_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_subscriber;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        float frequency = 20; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;
        float a=0.05;//Base distance
        float R=0.06; //Radius of the wheels

void calculatePose(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        
        Um = msg->data[0];
        dd1 = msg->data[1];
        dd2 = msg->data[2];


        //We calculate current robot speeds and orientation motor 
        tt_dot=Um*sin(d1-d2)/a; //sin(d1-d2)/a
        x_dot=(2*cos(d1)*cos(d2)*cos(tt) - sin(d1+d2)*sin(tt))*Um;
        y_dot=(2*cos(d1)*cos(d2)*sin(tt) + sin(d1+d2)*cos(tt))*Um;

        //We integrate the speeds over time (add each time we get a new value)
        x+=x_dot*period;
        y+=y_dot*period;
        tt+=tt_dot*period;
        d1+=dd1*period; //Delta 1
        d2+=dd2*period; //Delta 2
        

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
    // void getState(const sensor_msgs::msg::JointState::SharedPtr jointState){
    //     for(int i=0;i<4;i++){
    //         stateArray[i].id=jointState->name[i];
    //         stateArray[i].position=jointState->position[i];
    //         stateArray[i].speed=jointState->velocity[i];
    //     }
    // }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<transform_broadcaster>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}