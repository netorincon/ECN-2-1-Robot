#include <string>
#include <math.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_input/msg/state_vector.hpp>
#include <Eigen/Dense>

struct Point{
    float x=0;
    float y=0;
};

struct Pose{
    float x=0;
    float y=0;
    float theta=0;
};

struct Vector{
    float x=0;
    float y=0;
    float z=0;
};

struct Twist{
    Vector linear;
    Vector angular;
};

struct Motor{
    float position=0;
    float velocity=0;
    float acceleration=0;
    float effort=0;
    int id;
    std::string name;
    void setPosition(float angle){
        while( angle >=  M_PI ) angle -= 2*M_PI ;
        while( angle <  -M_PI  ) angle += 2*M_PI ;
        position=angle;
        return;
    }
    void setVelocity(float speed);
    void setEffort(float torque);
};

class Robot {
    public: 

        //We define the robot parameters
        Pose pose;
        Twist twist;
        Motor phi1;
        Motor phi2;
        Motor delta1;
        Motor delta2;
        Point ICRLocation;

        float v1;
        float v2;

        float wheel_radius;
        float chassis_length;
        float chassis_width;
        float wheel_distance;
        float mass; 
        Eigen::Matrix2f K_inv;

        Robot(float x, float y, float theta, float wheel_radius, float chassis_length, float chassis_width, float wheel_distance, float mass);
        Robot() = default;
        void resetPose();
        void setPose(float _x, float _y, float _theta);
        void setMotorPositions(float _phi1, float _phi2, float _delta1, float _delta2);
        void setMotorVelocities(float _phi1, float _phi2, float _delta1, float _delta2);
        void applyControlInput(float um, float _delta1, float _delta2, float _period);
        sensor_msgs::msg::JointState getJointStates();
        geometry_msgs::msg::TransformStamped getOdometry();
        geometry_msgs::msg::TransformStamped getICRTransform();
        control_input::msg::StateVector getStateVector();
        float limit_angle(float angle);
        float limit_phiSpeed(float speed);
        float limit_deltaSpeed(float speed);
        Eigen::Matrix2f getKInv(float e);
};