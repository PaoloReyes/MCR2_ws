import minichallenge2.utils.puzzlebot_kinematics as puzzlebot_kinematics
import numpy as np
import rclpy
import transforms3d as t3d

from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

class Localization(Node):
    def __init__(self):
        super().__init__('localization_node')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('localization_update_rate', 25.0)
        # Get the parameters
        r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        l = self.get_parameter('wheel_base').get_parameter_value().double_value
        update_rate = self.get_parameter('localization_update_rate').get_parameter_value().double_value

        # Subscribers
        self.create_subscription(Float32, '/wr', self.wr_callback, 10)
        self.create_subscription(Float32, '/wl', self.wl_callback, 10)
        
        # Publishers
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Timers
        self.create_timer(1.0/update_rate, self.localize_puzzlebot)
        
        # Node variables
        self.puzzlebot_kinematic_model = puzzlebot_kinematics.get_puzzlebot_kinematic_model(r, l)
        self.wheels_speeds = np.array([0., 0.])
        self.puzzlebot_pose = np.array([0., 0., 0.])
        self.last_time = self.get_clock().now()

        # Log the node start
        self.get_logger().info('Puzzlebot localization node started.')
        
    def wr_callback(self, msg):
        self.wheels_speeds[0] = msg.data

    def wl_callback(self, msg):
        self.wheels_speeds[1] = msg.data

    def localize_puzzlebot(self):
        # Get the linear and angular speeds from the wheels speeds and the kinematic model
        speeds = self.puzzlebot_kinematic_model @ self.wheels_speeds
        # Decompose the linear and angular speeds into [vx, vy, w]
        decomposed_speeds = puzzlebot_kinematics.speeds_decomposer(speeds[0], speeds[1], self.puzzlebot_pose[2])

        # Get the delta time
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9

        # Update the pose [x, y, theta] based on [vx, vy, w] and dt
        self.puzzlebot_pose[0] += decomposed_speeds[0] * dt
        self.puzzlebot_pose[1] += decomposed_speeds[1] * dt
        self.puzzlebot_pose[2] += decomposed_speeds[2] * dt
        # Normalize the angle in the range [-pi, pi]
        self.puzzlebot_pose[2] = np.arctan2(np.sin(self.puzzlebot_pose[2]), np.cos(self.puzzlebot_pose[2]))

        # Odom message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.puzzlebot_pose[0]
        odom_msg.pose.pose.position.y = self.puzzlebot_pose[1]
        odom_msg.pose.pose.position.z = 0.0
        q = t3d.euler.euler2quat(0.0, 0.0, self.puzzlebot_pose[2])
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]
        odom_msg.pose.pose.orientation.w = q[0]
        odom_msg.twist.twist.linear.x = speeds[0]
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = speeds[1]

        # Publish the odometry message
        self.odometry_publisher.publish(odom_msg)

        # Update the last time
        self.last_time = self.get_clock().now()

def main():
    rclpy.init()

    node = Localization()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().info(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()