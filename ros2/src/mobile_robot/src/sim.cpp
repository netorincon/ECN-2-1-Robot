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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_input/msg/slider_command.hpp>
#include <control_input/msg/control_input.hpp>
#include <control_input/msg/position_command.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <control_input/msg/position_command.hpp>

struct MotorState{
    std::string id;
    float position;
    float speed;
};

struct Point{
    float x;
    float y;
};

double limit_angle(double a){
    while( a >=  2*M_PI ) a -= 2*M_PI ;
    while( a <  0 ) a += 2*M_PI ;
    return a ;
}


class sim : public rclcpp::Node
{
    public :
        
        sim(rclcpp::NodeOptions options) : Node("sim", options)
        {
            declare_parameter("mode", "velocity");
            get_parameter("mode", mode);
            //Create transform broadcaster
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            /*
            We subscribe to the topic cmd_robot via cmd_susbscriber 
            It receives the input vector which includes [um, delta1dot and delta2dot].
            qDot (derivative of the configuration vector) is estimated using the S matrix obtained in the Kinematic model calculations
            times the um input. 

            Then the values are integrated over time to obtain the current state vector.
            
            */  
            if(mode=="position"){
                position_subscriber = this->create_subscription<control_input::msg::PositionCommand>(
                    "position_cmd", 10, std::bind(&sim::calculatePoseFromPositionCmd, this, std::placeholders::_1));
            }
            if(mode=="velocity" || mode=="controller"){
                control_input_subscriber = this->create_subscription<control_input::msg::ControlInput>(
                    "control_cmd", 10, std::bind(&sim::calculatePoseFromControlCmd, this, std::placeholders::_1));
            }

            joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            
        }
        //We initialize all variables of the state vector at 0
        float tt, x, y, phi1, phi2, phi1d, phi2d, beta1, beta2, Um, dd1, dd2, d1, d2, tt_dot, x_dot, y_dot=0;

    private :
        geometry_msgs::msg::TransformStamped transform_stamped_;
        geometry_msgs::msg::TransformStamped icr;
        rclcpp::Subscription<control_input::msg::PositionCommand>::SharedPtr position_subscriber;
        rclcpp::Subscription<control_input::msg::ControlInput>::SharedPtr control_input_subscriber;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        sensor_msgs::msg::JointState joint_state;
        tf2::Quaternion rotation;
        float frequency = 50; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;
        float a=0.08;//Base distance
        float R=0.033; //Radius of the wheels
        std::string mode; //Operating mode (position/velocity)
        Point ICRLocation;

        void calculatePoseFromPositionCmd(const control_input::msg::PositionCommand::SharedPtr msg){
                beta1=msg->d1;
                beta2=msg->d2+M_PI;
                d1=msg->d1;
                d2=msg->d2;
                phi1=0;
                phi2=0;
                publishTransforms(); 
            }

        void calculatePoseFromControlCmd(const control_input::msg::ControlInput::SharedPtr msg){
                Um = msg->um;
                dd1 = msg->delta1dot;
                dd2 = msg->delta2dot;

                //We calculate current robot speeds and orientation motor speeds
                d1+=dd1*period; //Delta 1
                d2+=dd2*period; //Delta 2

                tt_dot=Um*sin(d1-d2)/a; //sin(d1-d2)/a
                tt+=tt_dot*period;
                tt=limit_angle(tt);

                x_dot=(2*cos(d1)*cos(d2)*cos(tt) - sin(d1+d2)*sin(tt))*Um;
                y_dot=(2*cos(d1)*cos(d2)*sin(tt) + sin(d1+d2)*cos(tt))*Um;
                phi1d=2*cos(d2)*Um/R;
                phi2d=2*cos(d1)*Um/R;

                //We integrate the speeds over time (add each time we get a new value)
                x+=x_dot*period;
                y+=y_dot*period;

                beta1=d1;
                beta2=d2+M_PI;

                phi1+=phi1d*period;
                phi2+=phi2d*period;
                
                //We limit the angles to 2pi
                phi1=limit_angle(phi1);
                phi2=limit_angle(phi2);
                publishTransforms(); 
        }

        void calculateICR(){
            //We define alpha1 and alpha2 as temporary angles for ICR calculation
            float alpha1=d1+M_PI/2;
            float alpha2=d2+M_PI/2;
            alpha1=limit_angle(alpha1);
            alpha2=limit_angle(alpha2);

            float diff=alpha1-alpha2;
            float diffAbs=abs(diff);
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
            //RCLCPP_INFO(this->get_logger(), "Difference: '%f'", sin(diff));
            if(sin(diff)!=0){
                ICRLocation.x=a+2*a*(sin(alpha2)*cos(alpha1)/sin(diff));
                ICRLocation.y=2*a*(sin(alpha1)*sin(alpha2)/sin(diff));
            }

        }


        void publishTransforms(){
            //We now publish the joint states
            std::vector<std::string> names={"left_wheel_base_joint", "right_wheel_base_joint", "left_wheel_joint", "right_wheel_joint"};
            std::vector<double> posValues={beta2, beta1, phi2, phi1};
            std::vector<double> velValues={dd2, dd1, phi2d, phi1d};
            joint_state.name=names;
            joint_state.position = posValues;
           

            //Message that contains the chassis transform with respect to the world         
            transform_stamped_.header.frame_id = "world"; // Nom du repère fixe
            transform_stamped_.child_frame_id = "chassis"; // Nom du repère du robot
            transform_stamped_.transform.translation.x = x;
            transform_stamped_.transform.translation.y = y;
            transform_stamped_.transform.translation.z = 0;
            rotation.setRPY(0,0,tt);
            transform_stamped_.transform.rotation.x = rotation.getX();
            transform_stamped_.transform.rotation.y = rotation.getY();
            transform_stamped_.transform.rotation.z = rotation.getZ();
            transform_stamped_.transform.rotation.w = rotation.getW();
            
            //Message that contains the position of the ICR with respect to the robot chassis
            calculateICR();        
            icr.header.frame_id = "chassis"; // Nom du repère fixe
            icr.child_frame_id = "icr"; // Nom du repère du robot
            icr.transform.translation.x = ICRLocation.x;
            icr.transform.translation.y = ICRLocation.y;
            icr.transform.translation.z = 0;

            joint_state.header.stamp=this->now();
            joint_publisher->publish(joint_state);
            transform_stamped_.header.stamp = this->now();
            tf_broadcaster_->sendTransform(transform_stamped_);
            icr.header.stamp = this->now();
            tf_broadcaster_->sendTransform(icr);
        }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sim>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}