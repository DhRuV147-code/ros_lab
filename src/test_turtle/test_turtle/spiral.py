import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Spiral(Node):
    def __init__(self):
        super().__init__('spiral')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.draw_spiral)
        self.linear_speed = 1.0
        self.angular_speed = 3.0
        self.linear_increment = 0.02
        self.angular_increment = 0.005

    def draw_spiral(self):
        
        # Create a message to draw a Spiral
        # The message is to be published on '/turtle1/cmd_vel'
        # This topic accepts the data structure of 'Twist'
        # 'Twist' is available in 'geometry_msgs' interface package
        # Use v = w x r for direction of rotation and radius
        
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed

        # Publish the velocity command
        self.publisher_.publish(msg)

        # Increasing the linear velocity continuously
        self.linear_speed += self.linear_increment
        # Decreasing angular speed
        self.angular_speed -= self.angular_increment

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Spiral()
        rclpy.spin(node)

    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
