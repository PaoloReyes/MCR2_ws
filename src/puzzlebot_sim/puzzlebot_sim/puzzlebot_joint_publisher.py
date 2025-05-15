import rclpy
import numpy as np
import transforms3d as t3d
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PuzzlebotJointPublisher(Node):
    def __init__(self):
        super().__init__('puzzlebot_joint_publisher')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 0.1
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Puzzlebot Joint Publisher Node has been started.')

    def timer_callback(self):
        joints_state = JointState()
        joints_state.header.stamp = self.get_clock().now().to_msg()
        joints_state.name = ['wheel_r_joint', 'wheel_l_joint']
        joints_state.position = [0.0] * len(joints_state.name)
        joints_state.velocity = [0.0] * len(joints_state.name)
        joints_state.effort = [0.0] * len(joints_state.name)

        seconds = self.get_clock().now().nanoseconds/1e9

        joints_state.position[0] = seconds%(2 * np.pi)
        joints_state.position[1] = (2*np.pi)-seconds%(2 * np.pi)

        odom_base_footprint_transform = TransformStamped()
        odom_base_footprint_transform.header.stamp = self.get_clock().now().to_msg()
        odom_base_footprint_transform.header.frame_id = 'odom'
        odom_base_footprint_transform.child_frame_id = 'base_footprint'
        odom_base_footprint_transform.transform.translation.x = 0.0
        odom_base_footprint_transform.transform.translation.y = 0.0
        odom_base_footprint_transform.transform.translation.z = 0.0

        q = t3d.euler.euler2quat(0.0, 0.0, seconds%(2 * np.pi))

        odom_base_footprint_transform.transform.rotation.x = q[1]
        odom_base_footprint_transform.transform.rotation.y = q[2]
        odom_base_footprint_transform.transform.rotation.z = q[3]
        odom_base_footprint_transform.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(odom_base_footprint_transform)
        self.publisher.publish(joints_state)
    
def main():
    rclpy.init()
    
    node = PuzzlebotJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted. Shutting down...')
        if rclpy.ok():
            rclpy.shutdown()
    
    node.destroy_node()

if __name__ == '__main__':
    main()