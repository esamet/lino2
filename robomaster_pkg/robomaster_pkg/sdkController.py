import sys
sys.path.append('/home/kae/Desktop/RoboMaster-SDK/examples/plaintext_sample_code/RoboMasterEP/connection/network/')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import robot_connection
USB_DIRECT_CONNECTION_IP = '192.168.42.2'
robot = robot_connection.RobotConnection(USB_DIRECT_CONNECTION_IP)
if not robot.open():
    print('open fail')
    exit(1)
robot.send_data('command')

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('nodum')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        x=(msg.linear.x)
        y=(msg.linear.y)
        z=(msg.angular.z)
        z=z*-30
        if(x==0 and y==0 and z<=0.01):
            robot.send_data(f'chassis stop')
        else:
            robot.send_data(f'chassis speed x {x} y {y} z {z}')
def main(args=None):
    
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
