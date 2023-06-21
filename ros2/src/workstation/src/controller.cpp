/*Controller node

The objective of this node is to be able to simulate the robot's motion 
only via software before the controller is deployed to the real robot. 

This node subscribes to the command_input topic.
The command input should include [um, deltaDot1 and deltaDot2]

*/

#include <algorithm>
#include <Eigen/Dense>
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
#include <control_input/msg/control_input.hpp>
#include <control_input/msg/position_command.hpp>
#include <control_input/msg/state_vector.hpp>
#include <robot_library/robot.h>

using namespace std::chrono_literals;
using Eigen::MatrixXd;

float limit_deltaSpeed(float a){ //Limit delta rotation speeds to +-55rpm (The dynamixel MX28 MAXIMUM speed)
    float max=(55.0/60.0)*(2*M_PI); 
    if( a >=  max) {a =max;}
    else if(a<= -max) {a=-max;} 
    return a;
}

class controller : public rclcpp::Node
{
    public :
        
        controller(rclcpp::NodeOptions options) : Node("controller", options)
        {
            declare_parameter("frequency", 20);
            frequency = this->get_parameter("frequency").as_int();
            period=(1/frequency);
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            command_publisher = this->create_publisher<control_input::msg::ControlInput>("control_cmd", 10);
            state_subscriber=this->create_subscription<control_input::msg::StateVector>(
                            "state_vector", 10, std::bind(&controller::updateState, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(std::chrono::milliseconds(int(period*1000)), std::bind(&controller::calculateControlInput, this));
            //updateK();

            turtle4 = Robot();
        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        float um, dd1, dd2, x, y, phi1, phi2=0, d1Cmd = 0, d2Cmd = 0;
        float a=0.08;
        float e=0.1;
        float kp=0.7;
        float ki=0.0;
        float kd=0.0;
        float period;
        float frequency;
        float time=0;
        bool initialized=false;
        rclcpp::Publisher<control_input::msg::ControlInput>::SharedPtr command_publisher;
        rclcpp::Subscription<control_input::msg::StateVector>::SharedPtr state_subscriber;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        geometry_msgs::msg::TransformStamped transform_stamped_;
        control_input::msg::ControlInput command;
        Eigen::Matrix2f K;
        Eigen::Matrix2f K_inv;
        Eigen::Vector2f u;
        Eigen::Vector2f xr{1,1};
        Eigen::Vector2f xrprev;
        Eigen::Vector2f xrdot;
        Eigen::Vector2f xp;
        Eigen::Vector2f sumError{0,0};
        Eigen::Vector2f prevError{0,0};
        Eigen::Vector2f deltaError{0,0};
        Eigen::Vector2f Error{0,0};

        Robot turtle4;

    void calculateControlInput(){

        //YOUR CODE SHOULD START HERE
        //At the end you should be assignning a value to um, d1Cmd and d2Cmd.

        getNextPoint();
        xp(0)=turtle4.pose.x+(turtle4.wheel_distance/2)*cos(turtle4.pose.theta)+e*cos(turtle4.pose.theta+turtle4.delta1.position);
        xp(1)=turtle4.pose.y+(turtle4.wheel_distance/2)*sin(turtle4.pose.theta)+e*sin(turtle4.pose.theta+turtle4.delta1.position);
        u=turtle4.getKInv(e)*(xrdot+kp*(xr-xp));

        um=u(0);
        dd1=limit_deltaSpeed(u(1));
        // Change in position for d1 and d2
        d1Cmd = turtle4.delta1.position + (dd1 * period); 
        d2Cmd = -d1Cmd;

        //YOUR CODE SHOULD END HERE


        command.um=um;
        command.delta1 = d1Cmd;
        command.delta2 = d2Cmd;
        command_publisher->publish(command);

        transform_stamped_.header.stamp = this->now();
        transform_stamped_.header.frame_id = "odom"; // Name of the fixed frame
        transform_stamped_.child_frame_id = "target"; // Target frame
        transform_stamped_.transform.translation.x = xr(0);
        transform_stamped_.transform.translation.y = xr(1);
        transform_stamped_.transform.translation.z = 0;
        transform_stamped_.transform.rotation.x = 0;
        transform_stamped_.transform.rotation.y = 0;
        transform_stamped_.transform.rotation.z = 0;
        transform_stamped_.transform.rotation.w = 1;
        tf_broadcaster_->sendTransform(transform_stamped_);

        transform_stamped_.header.frame_id="odom";
        transform_stamped_.child_frame_id="p";
        transform_stamped_.transform.translation.x=xp(0);
        transform_stamped_.transform.translation.y=xp(1);
        tf_broadcaster_->sendTransform(transform_stamped_);
        return;


    }

    void updateState(const control_input::msg::StateVector::SharedPtr msg){
        turtle4.pose.x=msg->x;
        turtle4.pose.y=msg->y;
        turtle4.pose.theta=msg->theta;
        turtle4.delta1.position=msg->delta1;
        turtle4.delta2.position=msg->delta2;
        turtle4.phi1.position=msg->phi1;
        turtle4.phi2.position=msg->phi2;
        //updateK();
        initialized=true;
        return;
    }

    void getNextPoint(){
        time+=period/10;
        if(time>2*M_PI){
            time-=2*M_PI;
        }
        xr(0)=cos(time);
        xr(1)=sin(time);

        xrdot=(xr-xrprev)*period;
        xrprev=xr;

    
    }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
