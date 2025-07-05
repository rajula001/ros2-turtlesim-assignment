#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class TurtleAutonomy(Node):
    def __init__(self):
        super().__init__('turtle_autonomy')

        # Declare parameters for linear velocity (v) and timestep (dt)
        self.declare_parameter('v', 1.0)
        self.declare_parameter('dt', 0.5)

        # Initialize pose variables
        self.current_pose = None
        self.goal_pose = None

        # Subscribers
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Pose, '/goal_pose', self.goal_callback, 10)  # Custom goal topic

        # Publisher to command turtle movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.5, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg

    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None:
            return

        # Parameters
        v = self.get_parameter('v').get_parameter_value().double_value
        dt = self.get_parameter('dt').get_parameter_value().double_value

        # Stop if turtle is close enough to goal
        dx = self.goal_pose.x - self.current_pose.x
        dy = self.goal_pose.y - self.current_pose.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)
        if dist_to_goal < 0.3:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Reached goal. Stopping.")
            return

        # Try different angular velocities
        omega_values = [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0]
        best_omega = 0.0
        min_distance = float('inf')

        for omega in omega_values:
            theta_mid = self.current_pose.theta + (omega * dt / 2.0)
            x_pred = self.current_pose.x + v * dt * math.cos(theta_mid)
            y_pred = self.current_pose.y + v * dt * math.sin(theta_mid)

            dist = math.sqrt((self.goal_pose.x - x_pred)**2 + (self.goal_pose.y - y_pred)**2)

            if dist < min_distance:
                min_distance = dist
                best_omega = omega

        # Publish selected velocity command
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = best_omega
        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info(
            f"Moving with v={v:.2f}, omega={best_omega:.2f}, distance to goal={dist_to_goal:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TurtleAutonomy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
