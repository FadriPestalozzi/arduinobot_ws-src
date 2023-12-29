from launch import LaunchDescription
from launch_ros.actions import Node # from module actions in library launch_ros import class Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # name and directory of urdf model to be visualized in rviz
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduinobot.urdf.xacro"), # get_package_share_directory == full path
        description="Absolute path to the robot URDF file"
    )

    # directory converted from xacro file at model-path
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        parameters=[{"robot_description": robot_description}] # directory as argument to reuse for other urdf
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", 
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen", # show output
        arguments=["-d", os.path.join(get_package_share_directory("arduinobot_description"), "rviz", "display.rviz")] # pass directory
    )

    return LaunchDescription([
        model_arg, 
        robot_state_publisher, 
        joint_state_publisher_gui, 
        rviz_node
    ])