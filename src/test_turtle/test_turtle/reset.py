import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class Reset(Node):
    def __init__(self):
        super().__init__('reset')
                        
        # Create a client for the '/reset' service
        self.reset_client = self.create_client(Empty, '/reset')
        
        # Ensure the service is available
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for reset service...')
        
        self.reset_request()
        
    def reset_request(self):
        # Create the teleport request
        request = Empty.Request()
        
        # Send the request and wait for the service to complete
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)   
    
        # Check if the request succeeded
        if future.result() is not None:
            self.get_logger().info('TurtleSim has been reset.')
        else:
            self.get_logger().error('Failed to reset TurtleSim.')
            
def main(args=None):
    rclpy.init(args=args)
    try:
        node = Reset()
        rclpy.spin(node)

    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
