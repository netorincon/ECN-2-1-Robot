/*Real world node

The objective of this node is to obtain the joint speeds of the real robot and calculate its position in the real world
Then it calculates a transform and published it to /tf2 to be able to see the robot's motion in real time

This node subscribes to the command_input topic and motor_state topic.
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

class real_world : public rclcpp::Node
{
    public :
        real_world(rclcpp::NodeOptions options) : Node("real_world", options)
        {

            //Create transform broadcaster
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            /*
            We subscribe to the topic joint_states via state_susbscriber 

            This subscriber will receive a JointState type message, which will include a list of the states of each of the four motors. 
            (position, speed, and torque)

            To calculate current pose, we obtain (solve for) 'um' from the rows corresponding to PHI in the S(q) matrix. 
            Then we calculate the current twist using this um.

            Finally the values are integrated over time to obtain the current state vector.
            
            */

           //We update the transform each time we receive an state update
            state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                            "joint_states", 10, std::bind(&real_world::calculatePose, this, std::placeholders::_1));
            
        }
        //We initialize all variables of the state vector at 0
        float tt, x, y, phi1, phi2, phi1d, phi2d, beta1, beta2, Um, dd1, dd2, d1, d2, tt_dot, x_dot, y_dot=0;
        MotorState stateArray[4];

    private :
        geometry_msgs::msg::TransformStamped transform_stamped_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscriber;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        float frequency = 20; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;
        float a=0.05;//Base distance in meters
        float R=0.06; //Radius of the wheels in meters

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
        Um=phi1d*R/(2*cos(d2));

        //We calculate current robot speeds and orientation motor 
        tt_dot=Um*sin(d1-d2)/a; //sin(d1-d2)/a
        x_dot=(2*cos(d1)*cos(d2)*cos(tt) - sin(d1+d2)*sin(tt))*Um;
        y_dot=(2*cos(d1)*cos(d2)*sin(tt) + sin(d1+d2)*cos(tt))*Um;

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
    }

};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<real_world>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}