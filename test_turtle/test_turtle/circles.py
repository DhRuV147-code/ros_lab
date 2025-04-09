import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import math
import time

class CircleSquareDrawer(Node):
    def __init__(self):
        super().__init__('circle')
        
        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Define the centers of the circles (x, y coordinates)
        self.centers = [
            (5.5, 5.5),
            (1.0, 1.0),
            (4.0, 4.0),
            (3.0, 7.0)
        ]
                
        # Create a client for the teleport service
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        # Ensure the service is available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        
        # Start the process of drawing the circles and square
        self.draw_circles()
    
    def draw_circles(self):
        # Draw circles at the specified centers
        for center in self.centers:
            self.teleport_to(center[0], center[1])
            self.draw_circle()
                    
        self.get_logger().info("Completed drawing circles.")
    
    def teleport_to(self, x, y):
        # Create the teleport request
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        
        # Send the request and wait for the result
        future = self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    
    def draw_circle(self):
        # Create and publish a Twist message to draw a circle
        twist_msg = Twist()
        twist_msg.linear.x = 7.0  # Linear velocity
        twist_msg.angular.z = 7.0  # Angular velocity
        self.cmd_vel_pub.publish(twist_msg)
        time.sleep(1)
        # Stop the turtle after drawing the circle
        self.cmd_vel_pub.publish(Twist())
    
    
def main(args=None):
    rclpy.init(args=args)
    circle_square_drawer = CircleSquareDrawer()
    rclpy.spin(circle_square_drawer)
    circle_square_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
