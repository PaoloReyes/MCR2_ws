import rclpy
import transforms3d as t3d
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

class PuzzleDrone(Node):
    def __init__(self):
        super().__init__('puzzle_drone')
        self.get_logger().info('Puzzle Drone Node has been started.')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        timer_period = 0.1
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        time = self.get_clock().now()

        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = time.to_msg()
        odom_to_base_link.header.frame_id = 'odom'
        odom_to_base_link.child_frame_id = 'base_link'
        odom_to_base_link.transform.translation.x = np.sin(time.nanoseconds/4e9)
        odom_to_base_link.transform.translation.y = np.cos(time.nanoseconds/4e9)
        odom_to_base_link.transform.translation.z = 2.0
        q = t3d.euler.euler2quat(0.0, 0.0, 0.0)
        odom_to_base_link.transform.rotation.x = q[1]
        odom_to_base_link.transform.rotation.y = q[2]
        odom_to_base_link.transform.rotation.z = q[3]
        odom_to_base_link.transform.rotation.w = q[0]

        joint_state = JointState()
        joint_state.header.stamp = time.to_msg()
        joint_state.name = ['prop1_joint', 'prop2_joint', 'prop3_joint', 'prop4_joint']
        joint_state.position = [time.nanoseconds/1e9%(2*np.pi)] * 4
        joint_state.velocity = [0.0] * 4
        joint_state.effort = [0.0] * 4

        self.tf_broadcaster.sendTransform(odom_to_base_link)
        self.joint_state_publisher.publish(joint_state)

def main():
    rclpy.init()

    node = PuzzleDrone()

    try:
        rclpy.spin(node)
    except:
        node.get_logger().info('Shutting down...')
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
