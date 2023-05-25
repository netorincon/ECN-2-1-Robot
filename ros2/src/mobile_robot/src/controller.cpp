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
#include <control_input/msg/slider_command.hpp>
#include <control_input/msg/motor_command.hpp>
#include <control_input/msg/control_input.hpp>
#include <control_input/msg/position_command.hpp>
#include <control_input/msg/state_vector.hpp>
using namespace std::chrono_literals;
using Eigen::MatrixXd;

class controller : public rclcpp::Node
{
    public :
        
        controller(rclcpp::NodeOptions options) : Node("controller", options)
        {
            command_publisher = this->create_publisher<control_input::msg::ControlInput>("control_cmd", 10);
            state_subscriber=this->create_subscription<control_input::msg::StateVector>(
                            "state_vector", 10, std::bind(&controller::updateState, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(20ms, std::bind(&controller::calculateControlInput, this));
            updateK();
        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        float um, dd1, dd2, x, y, theta, d1, d2, phi1, phi2;
        float a=0.08;
        float e=0.01;
        float kp=0.01;
        float period=0.01;
        rclcpp::Publisher<control_input::msg::ControlInput>::SharedPtr command_publisher;
        rclcpp::Subscription<control_input::msg::StateVector>::SharedPtr state_subscriber;
        control_input::msg::ControlInput command;
        Eigen::Matrix2f K;
        Eigen::Matrix2f K_inv;
        Eigen::Vector2f u;
        Eigen::Vector2f xr;
        Eigen::Vector2f xrprev;
        Eigen::Vector2f xrdot;
        Eigen::Vector2f xp;


    void calculateControlInput(){

        //YOUR CODE SHOULD START HERE
        //At the end you should be assignning a value to um, dd1 and dd2.
        getNextPoint();
        xp(0)=x+a*cos(theta)+e*cos(theta+d1);
        xp(1)=y+a*sin(theta)+e*sin(theta+d1);

        u=K_inv*(xrdot+kp*(xr-xp));

        um=u(0);
        dd1=u(1);
        dd2=dd1;

        //YOUR CODE SHOULD END HERE

        command.um=um;
        command.delta1dot=dd1;
        command.delta2dot=dd2;
        command_publisher->publish(command);
        return;
    }

    void updateState(const control_input::msg::StateVector::SharedPtr msg){
        x=msg->x;
        y=msg->y;
        theta=msg->theta;
        d1=msg->delta1;
        d2=msg->delta2;
        phi1=msg->phi1;
        phi2=msg->phi2;
        updateK();
        return;
    }

    void updateK(){
        K(0,0)=cos(d1 + theta)/(2*sin(d1 + theta)*sin(d2 + theta)*cos(d1) + pow(2*cos(d2)*cos(d1 + theta),2));
        K(0,1)=sin(d1 + theta)/(2*sin(d1 + theta)*sin(d2 + theta)*cos(d1) + pow(2*cos(d2)*cos(d1 + theta),2));
        K(1,0)=(-2*a*sin(d2 + theta)*cos(d1) - e*sin(d1 - d2)*cos(d1 + theta))/(2*a*e*sin(d1 + theta)*sin(d2 + theta)*cos(d1) + pow(2*a*e*cos(d2)*cos(d1 + theta),2));
        K(1,1)=(2*a*cos(d2)*cos(d1 + theta) - e*sin(d1 - d2)*sin(d1 + theta))/(2*a*e*sin(d1 + theta)*sin(d2 + theta)*cos(d1) + pow(2*a*e*cos(d2)*cos(d1 + theta),2));
        K_inv=K.inverse();
    }

    void getNextPoint(){
        xr(0)=1;
        xr(1)=1;

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

