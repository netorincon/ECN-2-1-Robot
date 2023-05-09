#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


#include <chrono>



class transform_broadcaster : public rclcpp::Node
{
    public :
        transform_broadcaster(rclcpp::NodeOptions options) : Node("transform_broadcastor", options)
        {


            //init transform broadcastor
            tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            //init subscriber
            subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                            "cmd_robot", 10, std::bind(&transform_broadcaster::make_transforms, this, std::placeholders::_1));




        }
        float tt = 0, x = 0, y = 0;
        float a=0.05;

    private :
        geometry_msgs::msg::TransformStamped transform_stamped_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;








void make_transforms(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {

        float Um = msg->data[0];
        float bt1 = msg->data[1];
        float bt2 = msg->data[2];



        float tt_dot=(tan(bt2)-tan(bt1))*Um;
        float x_dot=(a*cos(tt)*(tan(bt1)+tan(bt2))+2*a*sin(tt))*Um;
        float y_dot=(a*sin(tt)*(tan(bt1)+tan(bt2))-2*a*sin(tt))*Um;

        float frequency = 20; // fréquence de publication des transformations sur le topic /tf
        float period = 1/frequency;

        tt += tt_dot*period;
        x += x_dot*period;
        y += y_dot*period;


        transform_stamped_.header.stamp = this->now();
        transform_stamped_.header.frame_id = "world"; // Nom du repère fixe
        transform_stamped_.child_frame_id = "chassis"; // Nom du repère du robot
        transform_stamped_.transform.translation.x += x_dot;
        transform_stamped_.transform.translation.y += y_dot;
        transform_stamped_.transform.translation.z += 0;
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
  rclcpp::spin(std::make_shared<transform_broadcaster>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}


