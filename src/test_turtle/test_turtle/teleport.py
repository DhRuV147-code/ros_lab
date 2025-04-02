import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
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
        
        self.teleport_to()
        
    def teleport_to(self):
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
            
def main(args=None):
    rclpy.init(args=args)
    try:
        node = Ports()
        rclpy.spin(node)

    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
