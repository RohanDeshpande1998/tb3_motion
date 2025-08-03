#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from math import atan2, hypot, sin, cos, pi
import matplotlib.pyplot as plt
from std_msgs.msg import Float64

K_RHO = 2.0
K_ALPHA = 4.0

class PurePursuitPID(Node):
    def __init__(self):
        super().__init__('pure_pursuit_pid')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tracking_error_pub = self.create_publisher(Float64, '/tracking_error', 10)
        self.orientation_error_pub = self.create_publisher(Float64, '/orientation_error', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Load trajectory
        traj_path = self.declare_parameter("trajectory_path", "/home/rohand/tb3_motion_ws/data/trajectory.npz").value
        data = np.load(traj_path)
        self.x_traj = data['x']
        self.y_traj = data['y']
        self.dt = float(data['dt'])

        # Internal state
        self.pose = None
        self.yaw = 0.0
        self.index = 0

        self.timer = self.create_timer(self.dt, self.run)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (p.x, p.y)
        self.yaw = yaw

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def next_trajectory_point(self, count):
        return self.x_traj[count], self.y_traj[count]

    def run(self):
        if self.pose is None or self.index >= len(self.x_traj):
            return

        lx, ly = self.next_trajectory_point(self.index)
        dx = lx - self.pose[0]
        dy = ly - self.pose[1]
        rho = hypot(dx, dy)
        alpha = atan2(dy, dx) - self.yaw
        alpha = atan2(sin(alpha), cos(alpha))
        
        # Compute control
        v = K_RHO * rho
        w = K_ALPHA * alpha
        if self.index >= len(self.x_traj) - 1:
            v = 0.0

        # Send velocity command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        
        # Publish errors
        self.tracking_error_pub.publish(Float64(data=rho))
        self.orientation_error_pub.publish(Float64(data=alpha))
        
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()