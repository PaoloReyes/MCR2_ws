import rclpy
import transforms3d as t3d
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MarkersPublisher(Node):
    def __init__(self):
        super().__init__('markers_publisher')
        self.get_logger().info('Markers Publisher Node has been started.')

        self.marker_publisher = self.create_publisher(Marker, '/marker', 10)

        self.omega = 0.5
        self.initial_pose_x = 0.0
        self.initial_pose_y = 0.0
        self.initial_pose_z = 1.0

        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.initial_pose_x
        self.marker.pose.position.y = self.initial_pose_y
        self.marker.pose.position.z = self.initial_pose_z
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.2
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        timer_period = 0.1
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        time = self.get_clock().now().nanoseconds/1e9

        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.pose.position.x = self.initial_pose_x + 0.5*np.sin(self.omega*time)
        self.marker.pose.position.y = self.initial_pose_y + 0.5*np.cos(self.omega*time)
        self.marker.pose.position.z = self.initial_pose_z

        q = t3d.euler.euler2quat(0.0, 1.57, self.omega*time)
        self.marker.pose.orientation.x = q[1]
        self.marker.pose.orientation.y = q[2]
        self.marker.pose.orientation.z = q[3]
        self.marker.pose.orientation.w = q[0]
        
        self.marker_publisher.publish(self.marker)

def main():
    rclpy.init()

    node = MarkersPublisher()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
