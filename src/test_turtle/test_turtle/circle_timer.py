import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CircleTimer(Node):
    def __init__(self):
        super().__init__('circle_timer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.draw_circle)
        # Check the timer with 2 and 5 seconds
        self.vel_x=2.5
        self.omega_z=1.0

    def draw_circle(self):
        msg = Twist()
        msg.linear.x = self.vel_x
        msg.angular.z = self.omega_z
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CircleTimer()
        rclpy.spin(node)

    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
