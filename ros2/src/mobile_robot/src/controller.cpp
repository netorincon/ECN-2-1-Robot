/*Controller node

The objective of this node is to be able to simulate the robot's motion 
only via software before the controller is deployed to the real robot. 

This node subscribes to the command_input topic.
The command input should include [um, deltaDot1 and deltaDot2]

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
using namespace std::chrono_literals;

class controller : public rclcpp::Node
{
    public :
        
        controller(rclcpp::NodeOptions options) : Node("controller", options)
        {
            command_publisher = this->create_publisher<control_input::msg::ControlInput>("control_cmd", 10);
            timer_ = this->create_wall_timer(20ms, std::bind(&controller::calculateControlInput, this));
        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<control_input::msg::ControlInput>::SharedPtr command_publisher;
        control_input::msg::ControlInput command;
        float um, dd1, dd2=0.0;

    void calculateControlInput(){
        //YOUR CODE SHOULD START HERE






        //YOUR CODE SHOULD END HERE

        command.um=um;
        command.delta1dot=dd1;
        command.delta2dot=dd2;
        command_publisher->publish(command);
        return;
    }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}

