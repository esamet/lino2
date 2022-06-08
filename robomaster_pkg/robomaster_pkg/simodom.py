import sys
sys.path.append('/home/kae/Desktop/RoboMaster-SDK/examples/plaintext_sample_code/RoboMasterEP/connection/network/')
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import robot_connection
import numpy as np
USB_DIRECT_CONNECTION_IP = '192.168.42.2'
robot = robot_connection.RobotConnection(USB_DIRECT_CONNECTION_IP)
if not robot.open():
    print('open fail')
    exit(1)
robot.send_data('command')
recv2 = robot.recv_ctrl_data(5)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def sdk_twist(self):
        robot.send_data('chassis speed ?')
        recv = robot.recv_ctrl_data(5)
        x= recv.decode("utf-8")
        x = x.split()
        return x
    def sdk_pose(self):
        robot.send_data('chassis position ?')
        recv = robot.recv_ctrl_data(5)
        x= recv.decode("utf-8")
        x = x.split()
        return x
    def sdk_attitude(self):
        robot.send_data('chassis attitude ?')
        recv = robot.recv_ctrl_data(5)
        x= recv.decode("utf-8")
        x = x.split()
        roll=float(x[1])
        roll = roll*0.0174532925
        pitch=float(x[0])
        pitch=pitch*0.0174532925
        yaw=float(x[2])
        yaw=yaw*-0.0174532925
        quaternionke=[]
        quaternionke.append(np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
        quaternionke.append(np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2))
        quaternionke.append(np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2))
        quaternionke.append(np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
       
        return quaternionke
    def timer_callback(self):
        msg = Odometry()

        sdk_twist=self.sdk_twist() #<x> <y> <z> <w1> <w2> <w3> <w4>:
        #print(sdk_twist[0])
        sdk_pose=self.sdk_pose() # <x> <y> <z>: x-axis position (m), y-axis position (m), and yaw angle (°)

        #print(sdk_twist[0])
        sdk_attitude=self.sdk_attitude() # <pitch> <roll> <yaw>: pitch-axis angle (°), roll-axis angle (°), and yaw-axis angle (°)

        #print(sdk_twist[0])
        
        msg.header.frame_id = "/map"
        msg.child_frame_id = "/base_link"
        msg.pose.pose.position.x=float(sdk_pose[0])
        msg.pose.pose.position.y=float(sdk_pose[1])
        msg.pose.pose.position.z=0.0
        msg.pose.pose.orientation.x=float(sdk_attitude[0])
        msg.pose.pose.orientation.y=float(sdk_attitude[1])
        msg.pose.pose.orientation.z=float(sdk_attitude[2])
        msg.pose.pose.orientation.w=float(sdk_attitude[3])

        msg.twist.twist.linear.x=float(sdk_twist[0])
        msg.twist.twist.linear.y=float(sdk_twist[1])
        msg.twist.twist.linear.z=0.0
        msg.twist.twist.angular.x=0.0
        msg.twist.twist.angular.y=0.0
        msg.twist.twist.angular.z=0.0 #float(sdk_twist[2])

        
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()