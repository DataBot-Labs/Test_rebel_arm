from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
import xacro
import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():
    # Get the package share directory for the robot description package
    share_dir = get_package_share_directory('rebel_arm_description')

    # Specify the path to the Xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'arm.xacro')
    # Process the Xacro file to get the robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # Path to the robot controllers configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rebel_arm_description"), "config", "rebel_arm_controllers.yaml",
        ]
    )

    # Declare the robot description as a parameter
    robot_description = {"robot_description": robot_urdf}

    # Define the control node for the controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],  # Removed robot_description here
        remappings=[
            ("~/robot_description", "/robot_description"),  # Remap the robot_description topic
        ],
        output="both",
    )
    
    # Define the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]  # Pass the robot description here
    )
    
    # Define the joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Define the robot controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"]
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node
    ])

