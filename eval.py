ros2 pkg create eval6_62 --build-type ament_python --dependencies rclpy
ros2 service call /spawn turtle/srv/spawn "{'x: 5', 'y: 5', 'theta: 0'}"
('/share') + package_name + ('/launch') , ['launch/turtle.launch.py']
ros2 run tf2_ros 


from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim', executable='turtlesim_node', output='screen', remappings=[('/turtle2/cmd_vel', 'turtle1/cmd_vel')]),
        launch_ros.actions.Node(
            package='turtlesim', executable='turtle_teleop_key', output='screen', arguments= 'x_term')
    ])