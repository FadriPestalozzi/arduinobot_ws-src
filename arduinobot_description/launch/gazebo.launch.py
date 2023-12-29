from launch import LaunchDescription
from launch_ros.actions import Node # from module actions in library launch_ros import class Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription # nest launch
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource # nest launch.py

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(get_package_share_directory("arduinobot_description"), "urdf", "arduinobot.urdf.xacro"), # get_package_share_directory == full path
        description="Absolute path to the robot URDF file"
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("arduinobot_description"), "share")) # access share folder

    # directory converted from xacro file at model-path
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        parameters=[{"robot_description": robot_description}] # directory as argument to reuse for other urdf
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")) # in launch folder 
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    )

    spawn_robot = Node(
        package="gazebo_ros", 
        executable="spawn_entity.py", 
        arguments=["-entity", "arduinobot", "-topic", "robot_description"]
    )

    # list of functionalities started with this launch file
    return LaunchDescription([
        env_var, 
        model_arg, # path of urdf model
        robot_state_publisher, 
        start_gazebo_server, 
        start_gazebo_client, 
        spawn_robot
    ])