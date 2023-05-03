// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>


#if defined(__linux__) || defined(__APPLE__)
  #include <fcntl.h>          // FILE control
  #include <termios.h>        // Terminal IO
#elif defined(_WIN32)
  #include <conio.h>
#endif

#define ESC_ASCII_VALUE             0x1b

// Keyboard QWERTY
/*#define FORWARD                     0x77	// w
#define BACKWARD                    0x78	// x
#define LEFT                        0x61	// a
#define RIGHT                       0x64	// d
#define STOPS                       0x73	// s*/

// Keyboard AZERTY
#define FORWARD                     0x7A	// z
#define BACKWARD                    0x78	// x
#define LEFT                        0x71	// q
#define RIGHT                       0x64	// d
#define STOPS                       0x73	// s


ros::Publisher get_position_pub;

// Receive characters by keyboard

int getch(void)
{
  #if defined(__linux__) || defined(__APPLE__)

    struct termios oldt, newt;
    int ch;

    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

    return ch;

  #elif defined(_WIN32) || defined(_WIN64)
    return _getch();
  #endif
}

int kbhit(void)
{
  #if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }
    return 0;
  #elif defined(_WIN32)
    return _kbhit();
  #endif
}

int main(int argc, char ** argv)
{
  // Receive characters by keyboard
  
  int32_t lin_vel_step = 0;
  int32_t ang_vel_step = 0;
   
  // Operating Mode 
  // 1: Velocity 
  // 3: Position   
  
  // Initialize node, service, subscribers and publishers
  
  ros::init(argc, argv, "read_write_node_keyboard");
  ros::NodeHandle nh;
  
  ros::ServiceServer get_velocity_srv = nh.advertiseService("/get_velocity", getPresentVelocityCallback);
  ros::Subscriber set_velocity_sub = nh.subscribe("/set_velocity", 10, setVelocityCallback);
  ros::Subscriber set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);
  
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);
  get_position_pub = nh.advertise<dynamixel_sdk_examples::Position>("/position_encoder", 10);
    
  // // Receive characters by keyboard
  
  std::string msg =
  "\n\
  Control Your Mobile Robot! \n\
  --------------------------- \n\
  Moving around:\n\
          z\n\
     q    s    d\n\
          x\n\
  \n\
  z/x : increase/decrease linear velocity\n\
  q/d : increase/decrease angular velocity\n\
  \n\
  s : force stop\n\
  \n\
  CTRL-C to quit\n\
  ";

  ROS_INFO("%s", msg.c_str());
  
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    if (kbhit())
    {
      char c = getch();

      if (c == FORWARD)
      {
        if (lin_vel_step >= 330)
          lin_vel_step = 330;
        else
          lin_vel_step = lin_vel_step + 2;
      }
      else if (c == BACKWARD)
      {
        if (lin_vel_step <= -330)
          lin_vel_step = -330;
        else
          lin_vel_step = lin_vel_step - 2;        
      }
      else if (c == LEFT)
      {
        if (ang_vel_step >= 230)
          ang_vel_step = 230;
        else
          ang_vel_step = ang_vel_step + 2;        
      }
      else if (c == RIGHT)
      {
        if (ang_vel_step <= -230)
          ang_vel_step = -230;
        else
          ang_vel_step = ang_vel_step - 2;      
      }
      else if (c == STOPS)
      {
        lin_vel_step = 0;
        ang_vel_step = 0;
      }      
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
