from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',executable='static_transform_publisher',output='screen',
            arguments=[
                '--frame-id', 'world',
                '--child-frame-id', 'base_link',
                '--x', '1.0',
                '--y', '2.0',
                '--z', '3.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0'
            ]
        )
    ])
