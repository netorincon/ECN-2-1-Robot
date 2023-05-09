#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <chrono>
#include <math.h>
#include <std_msgs/msg/float64_multi_array.hpp>






using namespace std::chrono_literals;


class driver : public rclcpp::Node
{
    public :
        driver(rclcpp::NodeOptions options) : Node("driver", options)
        {
            //init subscriber
            subscriber_=this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "cmd_robot", 10, std::bind(&driver::publishcommand, this, std::placeholders::_1)
            );

            // init publisher
            publisher_=this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);

            //init command
            //const std::vector<std::string> name = {"left_wheel", "left_wheel_base", "right_wheel", "right_wheel_base"};

            publisher_->publish(command);

            // init timer
            timer_=this->create_wall_timer(
                50ms, std::bind(&driver::timer_callback,this)
            );








        }

    private :
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Twist order;
        sensor_msgs::msg::JointState command;




        void timer_callback(){
            publisher_->publish(command);
        }

        void publishcommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg){

            // configuration kinematic model : dq/dt=S(q).u
            // u=[um us] where us=d(beta)/dt, beta=[beta_steerable beta_castor]
            // q=[posture beta phi], posture=[x y theta]

            float a{0.05}; // robot's geometric paramaters


            float Um=msg->data[0];
            float bt1=msg->data[1];
            float bt2=msg->data[2];




            float phi1{0};
            float phi2{0};


            phi1 += (-a*sin(bt1+bt2)*sin(bt1) - 2*a*pow(cos(bt1),2)*cos(bt2) + a*sin(bt1)*sin(bt2-bt1))*Um;
            phi2 += (a*sin(bt1+bt2)*sin(bt2) + 2*a*cos(bt1)*pow(cos(bt2),2) + a*sin(bt2)*sin(bt2-bt1))*Um;

            command.header.stamp=this->get_clock()->now();
            command.name = {"left_wheel_base_joint", "left_wheel_joint", "right_wheel_base_joint", "right_wheel_joint"};
            command.position = {bt2,phi2,bt1,phi1};




        }





        
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<driver>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
