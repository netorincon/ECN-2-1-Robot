#include <string>
#include <math.h>

struct Point{
    float x;
    float y;
};

class MobileRobot {
    public: 

        //We define the robot parameters
        float _x;
        float _y;
        float _theta;

        float _xd;
        float _yd;
        float _thetad;

        float _phi1;
        float _phi2;
        float _phi1d;
        float _phi2d;
        float _phi1dd;
        float _phi2dd;

        float _delta1;
        float _delta2;
        float _delta1d;
        float _delta2d;

        float _v1;
        float _v2;

        float _wheel_radius;
        float _chassis_length;
        float _chassis_width;
        float _wheel_distance;
        float _mass;

        Point ICRLocation;

        MobileRobot(float x, float y, float theta, float wheel_radius, float chassis_length, float chassis_width, float wheel_distance, float mass){
            _x=x;
            _y=y;
            _theta=theta;
            _xd=0;
            _yd=0;
            _thetad=0;
            _phi1=0;
            _phi2=0;
            _phi1d=0;
            _phi2d=0;
            _phi1dd=0;
            _phi2dd=0;
            _delta1=0;
            _delta2=0;
            _delta2d=0;
            _delta2d=0;
            _v1=0;
            _v2=0;

            _wheel_radius=wheel_radius;
            _chassis_length=chassis_length;
            _chassis_width=chassis_width;
            _wheel_distance=wheel_distance;
            _mass=mass;

        }

        void ResetPose(){
            _x=0;
            _y=0;
            _theta=0;
            _xd=0;
            _yd=0;
            _thetad=0;
            _phi1=0;
            _phi2=0;
            _phi1d=0;
            _phi2d=0;
            _delta1=0;
            _delta2=0;
            _delta2d=0;
            _delta2d=0;
            _phi1dd=0;
            _phi2dd=0;
            _v1=0;
            _v2=0;
        }


};