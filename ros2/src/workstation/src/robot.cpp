#include <robot.h>

Robot::Robot(float _x, float _y, float _theta, float _wheel_radius, float _chassis_length, float _chassis_width, float _wheel_distance, float _mass){
            pose.x=_x;
            pose.y=_y;
            pose.theta=_theta;
            wheel_radius=_wheel_radius;
            chassis_length=_chassis_length;
            chassis_width=_chassis_width;
            wheel_distance=_wheel_distance;
            mass=_mass;
            
            phi1.id=2;
            phi1.name="right_wheel_joint";

            phi2.id=1;
            phi2.name="left_wheel_joint";

            delta1.id=3;
            delta1.name="right_wheel_base_joint";

            delta2.id=4;
            delta2.name="left_wheel_base_joint";
}

void Robot::resetPose(){
            pose.x=0;
            pose.y=0;
            pose.theta=0;
            twist.linear.x=0;
            twist.linear.y=0;
            twist.angular.z=0;
            phi1.position=0;
            phi2.position=0;
            phi1.velocity=0;
            phi2.velocity=0;
            delta1.position=0;
            delta2.position=0;
            delta2.velocity=0;
            delta2.velocity=0;
            phi1.acceleration=0;
            phi2.acceleration=0;
            v1=0;
            v2=0;
        }

void Robot::setMotorPositions(float _phi1, float _phi2, float _delta1, float _delta2){
    phi1.position=_phi1;
    phi2.position=_phi2;
    delta1.position=_delta1;
    delta2.position=_delta2;
}
void Robot::setMotorVelocities(float _phi1, float _phi2, float _delta1, float _delta2){
    phi1.velocity=_phi1;
    phi2.velocity=_phi2;
    delta1.velocity=_delta1;
    delta2.velocity=_delta2;
    v1 = phi1.velocity * wheel_radius; //Tangent speed of wheel one
    v2 = v1 * cos(delta1.position) / cos(delta2.position);
    twist.angular.z = (1 / (wheel_distance)) * (v1 * sin(delta1.position) - v2 * sin(delta2.position));
}

void Robot::setPose(float _x, float _y, float _theta){
    pose.x=_x;
    pose.y=_y;
    pose.theta=_theta;
}