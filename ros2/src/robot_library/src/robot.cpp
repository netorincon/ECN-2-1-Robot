#include <robot.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <control_input/msg/state_vector.hpp>

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

float Robot::limit_angle(float angle){
    while( angle >=  2*M_PI ) angle -= 2*M_PI ;
    while( angle <  0 ) angle += 2*M_PI ;
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
    msg.position.push_back(delta2.position);

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

    float alpha1=delta1.position+M_PI/2;
    float alpha2=delta2.position+M_PI/2;
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
    if(sin(diff)!=0){
        ICRLocation.x=(wheel_distance/2)+wheel_distance*(sin(alpha2)*cos(alpha1)/sin(diff));
        ICRLocation.y=wheel_distance*(sin(alpha1)*sin(alpha2)/sin(diff));
    }

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