import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Line(Node):
    def __init__(self):
        super().__init__('line')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.vel_x=2.5        
        self.draw_line()

    def draw_line(self):
        msg = Twist()
        msg.linear.x = self.vel_x
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Line()
        rclpy.spin(node)
    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
