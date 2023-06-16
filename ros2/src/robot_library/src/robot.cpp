#include <robot.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <control_input/msg/state_vector.hpp>
#include <Eigen/Dense>

Robot::Robot(float _x, float _y, float _theta, float _wheel1_radius, float _wheel2_radius, float _chassis_length, float _chassis_width, float _wheel_distance, float _mass){
            pose.x=_x;
            pose.y=_y;
            pose.theta=_theta;
            wheel1_radius=_wheel1_radius;
            wheel2_radius=_wheel2_radius;
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

            wheel1_origin.x=wheel_distance/2;
            wheel1_origin.y=0;

            wheel2_origin.x=-wheel_distance/2;
            wheel2_origin.y=0;
}

Robot::Robot(){
            pose.x=0;
            pose.y=0;
            pose.theta=0;
            wheel1_radius=0.033;
            wheel2_radius=0.033;
            chassis_length=0.33;
            chassis_width=0.3;
            wheel_distance=0.16;
            mass=2;
           
            phi1.id=2;
            phi1.name="right_wheel_joint";

            phi2.id=1;
            phi2.name="left_wheel_joint";

            delta1.id=3;
            delta1.name="right_wheel_base_joint";

            delta2.id=4;
            delta2.name="left_wheel_base_joint";

            wheel1_origin.x=wheel_distance/2;
            wheel1_origin.y=0;

            wheel2_origin.x=-wheel_distance/2;
            wheel2_origin.y=0;
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
    v1 = phi1.velocity * wheel1_radius; //Tangent speed of wheel one
    v2 = phi2.velocity * wheel2_radius; //v1 * cos(delta1.position) / cos(delta2.position);
    twist.angular.z = sin(delta1.position - delta2.position)/(wheel_distance/2);//(1 / (wheel_distance)) * (v1 * sin(delta1.position) - v2 * sin(delta2.position));
}

void Robot::setPose(float _x, float _y, float _theta){
    pose.x=_x;
    pose.y=_y;
    pose.theta=_theta;
}
void Robot::applyControlInput(float um, float _delta1, float _delta2, float _period){
    delta1.setPosition(_delta1); //limit_deltaSpeed(msg->delta1);
    delta2.setPosition(_delta2); //limit_deltaSpeed(msg->delta2);

    twist.angular.z=um*sin(delta1.position-delta2.position)/(wheel_distance/2); //sin(d1-d2)/a
    pose.theta+=twist.angular.z*_period;
    pose.theta=limit_angle(pose.theta);

    twist.linear.x=um*(2*cos(delta1.position)*cos(delta2.position)*cos(pose.theta) - sin(delta1.position+delta2.position)*sin(pose.theta));
    twist.linear.y=um*(2*cos(delta1.position)*cos(delta2.position)*sin(pose.theta) + sin(delta1.position+delta2.position)*cos(pose.theta));
    phi1.velocity=limit_phiSpeed(2*cos(delta2.position)*um/wheel1_radius);
    phi2.velocity=limit_phiSpeed(2*cos(delta1.position)*um/wheel2_radius);

    //We integrate the speeds over time (add each time we get a new value)
    pose.x+=twist.linear.x*_period;
    pose.y+=twist.linear.y*_period;

    phi1.setPosition(phi1.position + phi1.velocity*_period);
    phi2.setPosition(phi2.position + phi2.velocity*_period);
}

Eigen::Matrix2f Robot::getKInv(float e){

        K_inv(0,0)=cos(delta1.position + pose.theta)/(2*sin(delta1.position + pose.theta)*sin(delta2.position + pose.theta)*cos(delta1.position) + 2*cos(delta2.position)*pow(cos(delta1.position + pose.theta),2));
        K_inv(0,1)=sin(delta1.position + pose.theta)/(2*sin(delta1.position + pose.theta)*sin(delta2.position + pose.theta)*cos(delta1.position) + 2*cos(delta2.position)*pow(cos(delta1.position + pose.theta),2));
        K_inv(1,0)=(-2*(wheel_distance/2)*sin(delta2.position + pose.theta)*cos(delta1.position) - e*sin(delta1.position - delta2.position)*cos(delta1.position + pose.theta))/(2*(wheel_distance/2)*e*sin(delta1.position + pose.theta)*sin(delta2.position + pose.theta)*cos(delta1.position) + 2*(wheel_distance/2)*e*cos(delta2.position)*pow(cos(delta1.position + pose.theta),2));
        K_inv(1,1)=(2*(wheel_distance/2)*cos(delta2.position)*cos(delta1.position + pose.theta) - e*sin(delta1.position - delta2.position)*sin(delta1.position + pose.theta))/(2*(wheel_distance/2)*e*sin(delta1.position + pose.theta)*sin(delta2.position + pose.theta)*cos(delta1.position) + 2*(wheel_distance/2)*e*cos(delta2.position)*pow(cos(delta1.position + pose.theta),2));

        return K_inv;
}

float Robot::limit_angle(float angle){
    while( angle >=  M_PI ) angle -= 2*M_PI ;
    while( angle <  -M_PI ) angle += 2*M_PI ;
    return angle;
}

sensor_msgs::msg::JointState Robot::getJointStates(){
    sensor_msgs::msg::JointState msg;
    msg.name.push_back(phi1.name);
    msg.name.push_back(phi2.name);
    msg.name.push_back(delta1.name);
    msg.name.push_back(delta2.name);

    msg.position.push_back(phi1.position);
    msg.position.push_back(phi2.position);
    msg.position.push_back(delta1.position);
    msg.position.push_back(limit_angle(delta2.position + M_PI));

    msg.velocity.push_back(phi1.velocity);
    msg.velocity.push_back(phi2.velocity);
    msg.velocity.push_back(delta1.velocity);
    msg.velocity.push_back(delta2.velocity);

    msg.effort.push_back(phi1.effort);
    msg.effort.push_back(phi2.effort);
    msg.effort.push_back(delta1.effort);
    msg.effort.push_back(delta2.effort);

    return msg;
}

geometry_msgs::msg::TransformStamped Robot::getOdometry(){
    geometry_msgs::msg::TransformStamped msg;
    tf2::Quaternion rotation;

    msg.header.frame_id = "odom"; // Nom du repère fixe
    msg.child_frame_id = "base_link"; // Nom du repère du robot
    msg.transform.translation.x = pose.x;
    msg.transform.translation.y = pose.y;
    msg.transform.translation.z = 0;
    rotation.setRPY(0,0,pose.theta);
    msg.transform.rotation.x = rotation.getX();
    msg.transform.rotation.y = rotation.getY();
    msg.transform.rotation.z = rotation.getZ();
    msg.transform.rotation.w = rotation.getW();

    return msg;
}

geometry_msgs::msg::TransformStamped Robot::getICRTransform(){

    auto alpha1=delta1.position+M_PI/2;
    auto alpha2=delta2.position+M_PI/2;

    auto c = tan(alpha1);
    auto d = tan(alpha2);

    float diff=tan(alpha1)-tan(alpha2);
        if(diff != 0){
            if((alpha1 != M_PI/2) && (alpha2 != M_PI/2)){
                ICRLocation.x=(wheel1_origin.y-wheel2_origin.y-(wheel_distance/2)*(c+d))/(d-c);
            }

            else if(alpha1==M_PI/2){
                ICRLocation.x=wheel_distance/2;
            }
            else if(alpha2==M_PI/2){
                ICRLocation.x=-wheel_distance/2;
            }
            ICRLocation.y=wheel1_origin.y - c*(wheel_distance/2) + c*ICRLocation.x;
    }

    // //See PDF for detailed calculations
    geometry_msgs::msg::TransformStamped icr;
    icr.header.frame_id = "base_link"; // Nom du repère fixe
    icr.child_frame_id = "icr"; // Nom du repère du robot
    icr.transform.translation.x = ICRLocation.x;
    icr.transform.translation.y = ICRLocation.y;
    icr.transform.translation.z = 0;

    return icr;
}

control_input::msg::StateVector Robot::getStateVector(){
    control_input::msg::StateVector msg;

    msg.x=pose.x;
    msg.y=pose.y;
    msg.theta=pose.theta;
    msg.delta1=delta1.position;
    msg.delta2=delta2.position;
    msg.phi1=phi1.position;
    msg.phi2=phi2.position;

    return msg;
}

float Robot::limit_phiSpeed(float speed){
    float max=(75.0/60.0)*(2*M_PI); //Limit phi rotation speeds to +-75rpm(The dynamixel XM430-W210 MAXIMUM speed)
    if( speed >=  max) {speed =max;}
    else if(speed<= -max) {speed=-max;} 
    return speed;
}

float limit_deltaSpeed(float speed){ //Limit delta rotation speeds to +-55rpm (The dynamixel MX28 MAXIMUM speed)
    float max=(55.0/60.0)*(2*M_PI);
    if( speed >=  max) {speed =max;}
    else if(speed<= -max) {speed=-max;} 
    return speed;
}