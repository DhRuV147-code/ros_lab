# turtlesim/launch/multisim.launch.py

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim', executable='turtlesim_node', output='screen', remappings=[('/turtle2/cmd_vel', 'turtle1/cmd_vel')])
    ])