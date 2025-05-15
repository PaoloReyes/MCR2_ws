import numpy as np
import rclpy

from rclpy.node import Node
from .utils import puzzlebot_kinematics
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PuzzlebotSim2(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim2')
        r = 0.05
        l = 0.19
        self.puzzlebot_inverse_kinematics_matrix = puzzlebot_kinematics.get_inverse_puzzlebot_kinematic_model(r, l)

        freq = 25
        self.create_timer(1.0/freq, self.timer_callback)

        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.wr_publisher = self.create_publisher(Float32, 'wr', 10)
        self.wl_publisher = self.create_publisher(Float32, 'wl', 10)

        self.noise_kp = 0.016
        self.velocities = np.array([0.0, 0.0])

    def timer_callback(self):
        wheel_velocities = self.puzzlebot_inverse_kinematics_matrix @ self.velocities

        wr_noisy = wheel_velocities[0] + np.random.normal(0, self.noise_kp*np.abs(wheel_velocities[0]))
        wl_noisy = wheel_velocities[1] + np.random.normal(0, self.noise_kp*np.abs(wheel_velocities[1]))

        self.wr_publisher.publish(Float32(data=wr_noisy))
        self.wl_publisher.publish(Float32(data=wl_noisy))

    def cmd_vel_callback(self, msg):
        self.velocities = np.array([msg.linear.x, msg.angular.z])

def main():
    rclpy.init()
    node = PuzzlebotSim2()
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
