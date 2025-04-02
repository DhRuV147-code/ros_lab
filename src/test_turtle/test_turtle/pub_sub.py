#Commented
import sys
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class CmdVelRemap(Node):
   def __init__(self):
      super().__init__('cmd_vel_remap')
      self.subscription = self.create_subscription(
         Twist,
         '/turtle1/cmd_vel',
         self.cmd_vel_callback,
         10
      )
      self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
   def cmd_vel_callback(self, msg:Twist):
      self.publisher.publish(msg)
def main(args=None):
   rclpy.init(args=args)
   node = CmdVelRemap()
   rclpy.spin(node)
   rclpy.shutdown()
if __name__ == '__main__':
   main()
