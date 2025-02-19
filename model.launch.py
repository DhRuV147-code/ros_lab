from launch import LaunchDescription
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('learning_tf_urdf'), 'urdf','robot.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type = str)

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',executable='robot_state_publisher', parameters = [{'robot_description': robot_description}], output='screen',
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',executable='joint_state_publisher_gui',output='screen',
        ),
        launch_ros.actions.Node(
            package='rviz2',executable='rviz2',output='screen',
        ),
    ])
