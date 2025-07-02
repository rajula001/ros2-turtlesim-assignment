import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlePose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class TurtleOdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.create_subscription(TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/turtle1/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def pose_callback(self, msg: TurtlePose):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = math.sin(msg.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(msg.theta / 2.0)

        odom.twist.twist.linear.x = msg.linear_velocity
        odom.twist.twist.angular.z = msg.angular_velocity

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(msg.theta / 2.0)
        t.transform.rotation.w = math.cos(msg.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()