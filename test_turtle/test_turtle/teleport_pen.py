import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class Ports(Node):
    def __init__(self):
        super().__init__('circle')
                        
        # Create a client
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        # Ensure the service is available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        
        # Define the centers of the circles (x, y coordinates)
        self.port_locations = [
            (5.5, 5.5),
            (1.0, 1.0),
            (4.0, 4.0),
            (3.0, 7.0)
        ]
        
        self.send_teleport_request()
        
        # Create clients to set the pen
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Ensure the services are available
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Pen services...')
        
        self.send_setpen_request()
        
    def send_teleport_request(self):
        for location in self.port_locations:
        # Create the teleport request
            request = TeleportAbsolute.Request()
            request.x = location[0]
            request.y = location[1]
            request.theta = 0.0
            time.sleep(2)
            # Send the request and wait for the result
            future = self.teleport_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
     
    def send_setpen_request(self, r=250, g=250, b=250, width=3, off=False):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = int(off)  # Pen off when teleporting

        future=self.pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
               
def main(args=None):
    rclpy.init(args=args)
    try:
        node = Ports()
        rclpy.spin(node)

    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
