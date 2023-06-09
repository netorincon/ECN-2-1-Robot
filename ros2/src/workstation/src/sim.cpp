/*Simulation node

The objective of this node is to be able to simulate the robot's motion 
only via software before the controller is deployed to the real robot. 

This node subscribes to the command_input topic.
The command input should include [um, deltaDot1 and deltaDot2]

*/

/*
We subscribe to the topic cmd_robot via cmd_susbscriber 
It receives the input vector which includes [um, delta1dot and delta2dot].
qDot (derivative of the configuration vector) is estimated using the S matrix obtained in the Kinematic model calculations
times the um input. 

Then the values are integrated over time to obtain the current state vector.
*/

#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <control_input/msg/state_vector.hpp>
#include <control_input/msg/control_input.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_input/msg/position_command.hpp>
#include <robot_library/robot.h>

double limit_angle(double angle){
    while( angle >=  2*M_PI ) angle -= 2*M_PI ;
    while( angle <  0 ) angle += 2*M_PI ;
    return angle ;
}

float limit_phiSpeed(float angle){
    float max=(75.0/60.0)*(2*M_PI); //Limit phi rotation speeds to +-75rpm(The dynamixel XM430-W210 MAXIMUM speed)
    if( angle >=  max) {angle =max;}
    else if(angle<= -max) {angle=-max;} 
    return angle;
}

float limit_deltaSpeed(float angle){ //Limit delta rotation speeds to +-55rpm (The dynamixel MX28 MAXIMUM speed)
    float max=(55.0/60.0)*(2*M_PI);
    if( angle >=  max) {angle =max;}
    else if(angle<= -max) {angle=-max;} 
    return angle;
}

class sim : public rclcpp::Node
{
    public :
        
        sim(rclcpp::NodeOptions options) : Node("sim", options)
        {
            declare_parameter("frequency", 20);
            frequency = this->get_parameter("frequency").as_int();
            period = 1.0/frequency; //Seconds
            //Create transform broadcaster
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            position_subscriber = this->create_subscription<control_input::msg::PositionCommand>(
                "position_cmd", 10, std::bind(&sim::calculatePoseFromPositionCmd, this, std::placeholders::_1));

            control_input_subscriber = this->create_subscription<control_input::msg::ControlInput>(
                "control_cmd", 10, std::bind(&sim::calculatePoseFromControlCmd, this, std::placeholders::_1));

            joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            state_vector_publisher=this->create_publisher<control_input::msg::StateVector>("state_vector", 10);

            turtle4=Robot(0.0, 0, 0, 0.033, 0.33, 0.3, 0.16, 2);
            
        }
        //We initialize all variables of the state vector at 0

    private :

        float period;
        float frequency;
        Robot turtle4;
        geometry_msgs::msg::TransformStamped icr;
        sensor_msgs::msg::JointState joint_state;
        control_input::msg::StateVector robot_state;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
        rclcpp::Publisher<control_input::msg::StateVector>::SharedPtr state_vector_publisher;
        rclcpp::Subscription<control_input::msg::PositionCommand>::SharedPtr position_subscriber;
        rclcpp::Subscription<control_input::msg::ControlInput>::SharedPtr control_input_subscriber;

        void calculatePoseFromPositionCmd(const control_input::msg::PositionCommand::SharedPtr msg){
                turtle4.delta1.position=msg->d1;
                turtle4.delta2.position=msg->d2;
                turtle4.phi1.position=0;
                turtle4.phi2.position=0;
                publishTransforms(); 
            }

        void calculatePoseFromControlCmd(const control_input::msg::ControlInput::SharedPtr msg){
                turtle4.delta1.position = msg->delta1; //limit_deltaSpeed(msg->delta1);
                turtle4.delta2.position = msg->delta2; //limit_deltaSpeed(msg->delta2);

                turtle4.twist.angular.z=msg->um*sin(turtle4.delta1.position-turtle4.delta2.position)/(turtle4.wheel_distance/2); //sin(d1-d2)/a
                turtle4.pose.theta+=turtle4.twist.angular.z*period;
                turtle4.pose.theta=limit_angle(turtle4.pose.theta);

                turtle4.twist.linear.x=msg->um*(2*cos(turtle4.delta1.position)*cos(turtle4.delta2.position)*cos(turtle4.pose.theta) - sin(turtle4.delta1.position+turtle4.delta2.position)*sin(turtle4.pose.theta));
                turtle4.twist.linear.y=msg->um*(2*cos(turtle4.delta1.position)*cos(turtle4.delta2.position)*sin(turtle4.pose.theta) + sin(turtle4.delta1.position+turtle4.delta2.position)*cos(turtle4.pose.theta));
                turtle4.phi1.velocity=limit_phiSpeed(2*cos(turtle4.delta2.position)*msg->um/turtle4.wheel_radius);
                turtle4.phi2.velocity=limit_phiSpeed(2*cos(turtle4.delta1.position)*msg->um/turtle4.wheel_radius);

                //We integrate the speeds over time (add each time we get a new value)
                turtle4.pose.x+=turtle4.twist.linear.x*period;
                turtle4.pose.y+=turtle4.twist.linear.y*period;

                turtle4.phi1.position+=turtle4.phi1.velocity*period;
                turtle4.phi2.position+=turtle4.phi2.velocity*period;
                
                //We limit the angles to 2pi
                turtle4.phi1.position=limit_angle(turtle4.phi1.position);
                turtle4.phi2.position=limit_angle(turtle4.phi2.position);
                publishTransforms(); 
        }
        void publishTransforms(){

            //We now publish the joint states
            joint_state = turtle4.getJointStates();
            joint_state.header.stamp=this->now();
            joint_publisher->publish(joint_state);


            //Transform that contains the chassis transform with respect to the odom frame  
            transform_stamped_=turtle4.getOdometry();
            transform_stamped_.header.stamp = this->now();
            tf_broadcaster_->sendTransform(transform_stamped_);

            //Transform that contains the position of the ICR with respect to the robot chassis
            icr=turtle4.getICRTransform();
            icr.header.stamp = this->now();
            tf_broadcaster_->sendTransform(icr);

            state_vector_publisher->publish(turtle4.getStateVector());
        }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sim>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}