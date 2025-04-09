import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import math
import time

class Cycle(Node):

    def __init__(self):
        super().__init__('cycle_node')
        
        # Create publisher to publish the velocity commands
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create clients to move absolute using teleport
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        # Ensure the services are available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Teleport service ...')

        # Create clients to set the pen
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Ensure the services are available
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Pen services...')
        
        # Draw the drone
        self.draw_cycle()

    def set_pen(self, r=250, g=250, b=250, width=3, off=False):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = int(off)  # Pen off when teleporting

        future=self.pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
    def send_teleport_request(self, x, y, theta=0.0):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future=self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
    def draw_cycle(self):
        
        # Draw wheel (2 circles)
        self.draw_wheels()
        
        # Draw Square Seat
        self.draw_square()

        # Create frame to connect Seat and Wheels
        self.draw_frame()
        
        # Move the turtle to the center of the square without trace
        self.set_pen(off=True)  # Turn off pen
        self.send_teleport_request(5.5, 5.5)

    def draw_wheels(self):
        
        # Send turtle @ lowest point on the circumference : Circle (2,2)
        self.teleport(3.0, 2.0, off=True)
        self.draw_circle(False, 1.0)
        
        self.teleport(8.0, 2.0, off=True)
        self.draw_circle(False, 1.0)

    def draw_circle(self, trace, radius=1.0, speed=2.0):
        
        self.set_pen(off=trace)  # Turn on pen
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = speed / radius
        start_time = self.get_clock().now()

        # Draw the circle for 2π radians
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 2 * math.pi / twist.angular.z:
            self.publisher_.publish(twist)

        # Stop the turtle after the circle is drawn
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
    
    def draw_square(self):
        # Draw the drone frame (square)
        self.teleport(5.0, 5.0, off=True)
        
        self.teleport(6.0, 5.0, g=0)  
        self.teleport(6.0, 5.4)  
        self.teleport(5.0, 5.4, b=0)  
        self.teleport(5.0, 5.0)
    
    def draw_frame(self):
        
        # Move to the mid of the Seat
        self.teleport(5.5, 5.0, off=True)  
        # Go to Front wheel Center
        self.teleport(3.0, 3.0, r=140, g=100, b=20, off=False)  
        
        # Back wheel-Seat Frame
        self.teleport(5.5, 5.0, off=True) 
        self.teleport(8.0, 3.0, r=140, g=100, b=20, off=False) 
        
        # Front Column
        self.teleport(3.5, 6.0, off=True) 
        self.teleport(3.0, 3.0, off=False)  
        
        # Handle
        self.teleport(3.7, 6.1, off=True) 
        self.teleport(3.3, 5.9, r=140, g=100, b=120, off=False)  
        
    def teleport(self, x, y, r=250, g=250, b=250, off=False):
        self.set_pen(r=r, g=g, b=b, off=off)  # Set off
        time.sleep(1)  # Give time for the teleport to complete
        self.send_teleport_request(x, y)
        time.sleep(1)  # Give time for the teleport to complete

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Cycle()
        rclpy.spin(node)

    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
