import minichallenge3.utils.puzzlebot_kinematics as puzzlebot_kinematics
import numpy as np
import rclpy
import transforms3d as t3d

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PuzzlebotSim(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim')

        # Robot parameters declaration
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('simulation_update_rate', 25.0)
        # Get the parameters
        r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        l = self.get_parameter('wheel_base').get_parameter_value().double_value
        update_rate = self.get_parameter('simulation_update_rate').get_parameter_value().double_value

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.puzzlebot_pose_publisher = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.right_wheel_speed_publisher = self.create_publisher(Float32, 'wr', 10)
        self.left_wheel_speed_publisher = self.create_publisher(Float32, 'wl', 10)

        # Timers
        self.create_timer(1.0/update_rate, self.update_puzzlebot)
        
        # Node variables
        self.inverse_puzzlebot_kinematic_model = puzzlebot_kinematics.get_inverse_puzzlebot_kinematic_model(r, l)
        self.speeds = np.array([0., 0.])
        self.wheels_speeds = np.array([0., 0.])
        self.puzzlebot_pose = np.array([0., 0., 0.])
        self.last_time = self.get_clock().now()

        # Log the node start
        self.get_logger().info('Puzzlebot simulation node started.')

    def cmd_vel_callback(self, msg):
        # Update the robot speeds
        self.speeds[0] = msg.linear.x
        self.speeds[1] = msg.angular.z

        # Obtain the wheel speeds based on the inverse kinematic model
        self.wheels_speeds = self.inverse_puzzlebot_kinematic_model @ np.array([self.speeds[0], self.speeds[1]])

        # Float32 messages to pubish wheel speeds
        wr_msg = Float32()
        wl_msg = Float32()
        wr_msg.data = self.wheels_speeds[0]
        wl_msg.data = self.wheels_speeds[1]

        # Publish the wheel speeds
        self.right_wheel_speed_publisher.publish(wr_msg)
        self.left_wheel_speed_publisher.publish(wl_msg)

    def update_puzzlebot(self):
        # Decompose linear and angular speeds into vx, vy, and w
        speeds = puzzlebot_kinematics.speeds_decomposer(self.speeds[0], self.speeds[1], self.puzzlebot_pose[2])
        
        # Get the delta time
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9

        # Update the pose in the form [x, y, theta] using [vx, vy, w] and dt
        self.puzzlebot_pose[0] += speeds[0] * dt
        self.puzzlebot_pose[1] += speeds[1] * dt
        self.puzzlebot_pose[2] += speeds[2] * dt
        # Normalize the angle in the range [-pi, pi]
        self.puzzlebot_pose[2] = np.arctan2(np.sin(self.puzzlebot_pose[2]), np.cos(self.puzzlebot_pose[2]))

        # Create the pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = self.puzzlebot_pose[0]
        pose_msg.pose.position.y = self.puzzlebot_pose[1]
        pose_msg.pose.position.z = 0.0
        q = t3d.euler.euler2quat(0, 0, self.puzzlebot_pose[2])
        pose_msg.pose.orientation.x = q[1]
        pose_msg.pose.orientation.y = q[2]
        pose_msg.pose.orientation.z = q[3]
        pose_msg.pose.orientation.w = q[0]

        # Publish the new pose
        self.puzzlebot_pose_publisher.publish(pose_msg)

        # Update the last time
        self.last_time = self.get_clock().now()

def main():
    rclpy.init()

    node = PuzzlebotSim()

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
