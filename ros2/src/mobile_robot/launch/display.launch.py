import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions  import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import xacro


def generate_launch_description():

    # Check if we are told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('mobile_robot'))
    xacro_file = os.path.join(pkg_path,'description',"robot.urdf.xacro")
    #robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, " use_ros2_control:=", use_ros2_control, " sim_time:=", use_sim_time])

    # Create robot_state_publisher node
    params1 = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [params1]
    )

    # Create Rviz node
    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        output = 'screen',
    )

    # Create joint_state_publisher_gui node
    joint_state_publisher_gui = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
    )
    

			
    # Create slider_publisher node
    yaml_file = os.path.join(pkg_path,'launch','Twist.yaml')
    slider_publisher = Node(
		package = 'slider_publisher',
		executable = 'slider_publisher',
		name = 'slider_publisher',
		output = 'screen',
		arguments = [yaml_file],
	)
	
	# Create driver node
    driver = Node(
        package = 'mobile_robot',
        executable = 'driver',
        output = 'screen',
    )
    
    # Create transform_broadcaster node
    transform_broadcaster = Node(
        package = 'mobile_robot',
        executable = 'transform_broadcaster',
        output = 'screen',
    )

   



    # Launch
    return LaunchDescription([
    
			
			
        DeclareLaunchArgument(
            "use_sim_time",
            default_value = "false",
            description = 'use sim time if true '),
            
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value = "true",
            description = 'use ros2 control if true'),
        
        #joint_state_publisher_gui,
        robot_state_publisher,
        slider_publisher,
        driver,
        transform_broadcaster,
        rviz
        
        
        
    ])

