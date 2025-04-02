import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        
        self.static_broadcaster = StaticTransformBroadcaster(self)

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'
        static_transform.child_frame_id = 'robot'

        static_transform.transform.translation.x = 1.0
        static_transform.transform.translation.y = 2.0
        static_transform.transform.translation.z = 0.0

        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.707
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 0.707

        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Broadcasting static transform from "world" to "robot".')

def main():
    rclpy.init()
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
