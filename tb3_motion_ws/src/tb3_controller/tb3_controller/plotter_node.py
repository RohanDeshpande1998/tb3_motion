#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class ErrorPlotter(Node):
    def __init__(self):
        super().__init__('error_plotter')
        
        # Parameters
        traj_path = self.declare_parameter("trajectory_path", "/home/rohand/tb3_motion_ws/data/trajectory.npz").value
        data = np.load(traj_path)
        self.x_traj = data['x']
        self.y_traj = data['y']

        # Error data
        self.tracking_error = []
        self.orientation_error = []

        # Path messages
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)
        
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', latched_qos)
        
        self.actual_path = Path()
        self.actual_path.header.frame_id = "odom"

        # Matplotlib figure setup
        self.fig, self.axs = plt.subplots(2, 1)
        self.axs[0].set_title("Tracking Error")
        self.axs[1].set_title("Orientation Error")
        plt.ion()
        plt.show(block=False)

        # Subscriptions
        self.create_subscription(Float64, '/tracking_error', self.tracking_cb, 10)
        self.create_subscription(Float64, '/orientation_error', self.orientation_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Timer
        self.timer = self.create_timer(0.2, self.update)

        # Publish planned path once
        self.publish_planned_path()

    def tracking_cb(self, msg):
        self.tracking_error.append(msg.data)

    def orientation_cb(self, msg):
        self.orientation_error.append(msg.data)

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position = pos
        pose.pose.orientation = ori

        self.actual_path.poses.append(pose)
        self.actual_path.header.stamp = pose.header.stamp
        self.actual_path_pub.publish(self.actual_path)

    def publish_planned_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in zip(self.x_traj, self.y_traj):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation
            path_msg.poses.append(pose)

        self.planned_path_pub.publish(path_msg)

    def update(self):
        # Update error plots
        self.axs[0].clear()
        self.axs[0].plot(self.tracking_error, label='Tracking Error')
        self.axs[0].legend()
        self.axs[1].clear()
        self.axs[1].plot(self.orientation_error, label='Orientation Error')
        self.axs[1].legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = ErrorPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()