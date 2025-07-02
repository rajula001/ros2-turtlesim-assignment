import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from transforms3d.euler import euler2quat
from builtin_interfaces.msg import Time

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')

        # Dynamic broadcaster for odom -> base_link
        self.br = TransformBroadcaster(self)

        # Static broadcaster for map -> odom
        self.static_br = StaticTransformBroadcaster(self)
        self.send_static_transform()

        # Subscribe to /turtle1/pose
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.handle_turtle_pose,
            10
        )

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Convert Euler (roll=0, pitch=0, yaw=theta) to quaternion
        qw, qx, qy, qz = euler2quat(0, 0, msg.theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

    def send_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 5.5
        t.transform.translation.y = 5.5
        t.transform.translation.z = 0.0

        # No rotation between map and odom → use identity quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    rclpy.spin(node)
    rclpy.shutdown()
