import rclpy
import numpy as np
import transforms3d as t3d
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

class PuzzleDrone(Node):
    def __init__(self):
        super().__init__('puzzledrone')
        self.get_logger().info('Puzzle Drone Node has been started.')

        self.marker_publisher = self.create_publisher(Marker, '/marker', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 0.1
        self.create_timer(timer_period, self.timer_callback)

        self.omega = 0.25

    def timer_callback(self):
        odom_base_link_transform = TransformStamped()
        odom_base_link_transform.header.stamp = self.get_clock().now().to_msg()
        odom_base_link_transform.header.frame_id = 'odom'
        odom_base_link_transform.child_frame_id = 'base_link'
        odom_base_link_transform.transform.translation.x = np.sin(2*np.pi*self.omega*self.get_clock().now().nanoseconds/1e9)
        odom_base_link_transform.transform.translation.y = np.cos(2*np.pi*self.omega*self.get_clock().now().nanoseconds/1e9)
        odom_base_link_transform.transform.translation.z = 1.0
        odom_base_link_transform.transform.rotation.x = 0.0
        odom_base_link_transform.transform.rotation.y = 0.0
        odom_base_link_transform.transform.rotation.z = 0.0
        odom_base_link_transform.transform.rotation.w = 1.0

        odom_base_footprint_transform = TransformStamped()
        odom_base_footprint_transform.header.stamp = self.get_clock().now().to_msg()
        odom_base_footprint_transform.header.frame_id = 'odom'
        odom_base_footprint_transform.child_frame_id = 'base_footprint'
        odom_base_footprint_transform.transform.translation.x = np.sin(2*np.pi*self.omega*self.get_clock().now().nanoseconds/1e9)
        odom_base_footprint_transform.transform.translation.y = np.cos(2*np.pi*self.omega*self.get_clock().now().nanoseconds/1e9)
        odom_base_footprint_transform.transform.translation.z = 0.0
        odom_base_footprint_transform.transform.rotation.x = 0.0
        odom_base_footprint_transform.transform.rotation.y = 0.0
        odom_base_footprint_transform.transform.rotation.z = 0.0
        odom_base_footprint_transform.transform.rotation.w = 1.0

        q = t3d.euler.euler2quat(0.0, 0.0, 2*np.pi*self.omega*self.get_clock().now().nanoseconds/1e9)

        base_link_prop_1_transform = TransformStamped()
        base_link_prop_1_transform.header.stamp = self.get_clock().now().to_msg()
        base_link_prop_1_transform.header.frame_id = 'base_link'
        base_link_prop_1_transform.child_frame_id = 'prop_1'
        base_link_prop_1_transform.transform.translation.x = 0.06717
        base_link_prop_1_transform.transform.translation.y = -0.082
        base_link_prop_1_transform.transform.translation.z = -0.0125
        base_link_prop_1_transform.transform.rotation.x = q[1]
        base_link_prop_1_transform.transform.rotation.y = q[2]
        base_link_prop_1_transform.transform.rotation.z = q[3]
        base_link_prop_1_transform.transform.rotation.w = q[0]

        base_link_prop_2_transform = TransformStamped()
        base_link_prop_2_transform.header.stamp = self.get_clock().now().to_msg()
        base_link_prop_2_transform.header.frame_id = 'base_link'
        base_link_prop_2_transform.child_frame_id = 'prop_2'
        base_link_prop_2_transform.transform.translation.x = 0.06717
        base_link_prop_2_transform.transform.translation.y = 0.082
        base_link_prop_2_transform.transform.translation.z = -0.0125
        base_link_prop_2_transform.transform.rotation.x = q[1]
        base_link_prop_2_transform.transform.rotation.y = q[2]
        base_link_prop_2_transform.transform.rotation.z = q[3]
        base_link_prop_2_transform.transform.rotation.w = q[0]

        base_link_prop_3_transform = TransformStamped()
        base_link_prop_3_transform.header.stamp = self.get_clock().now().to_msg()
        base_link_prop_3_transform.header.frame_id = 'base_link'
        base_link_prop_3_transform.child_frame_id = 'prop_3'
        base_link_prop_3_transform.transform.translation.x = -0.06717
        base_link_prop_3_transform.transform.translation.y = -0.082
        base_link_prop_3_transform.transform.translation.z = -0.0125
        base_link_prop_3_transform.transform.rotation.x = q[1]
        base_link_prop_3_transform.transform.rotation.y = q[2]
        base_link_prop_3_transform.transform.rotation.z = q[3]
        base_link_prop_3_transform.transform.rotation.w = q[0]

        base_link_prop_4_transform = TransformStamped()
        base_link_prop_4_transform.header.stamp = self.get_clock().now().to_msg()
        base_link_prop_4_transform.header.frame_id = 'base_link'
        base_link_prop_4_transform.child_frame_id = 'prop_4'
        base_link_prop_4_transform.transform.translation.x = -0.06717
        base_link_prop_4_transform.transform.translation.y = 0.082
        base_link_prop_4_transform.transform.translation.z = -0.0125
        base_link_prop_4_transform.transform.rotation.x = q[1]
        base_link_prop_4_transform.transform.rotation.y = q[2]
        base_link_prop_4_transform.transform.rotation.z = q[3]
        base_link_prop_4_transform.transform.rotation.w = q[0]

        base_210mm_marker = Marker()
        base_210mm_marker.header.frame_id = 'base_link'
        base_210mm_marker.header.stamp = self.get_clock().now().to_msg()
        base_210mm_marker.id = 0
        base_210mm_marker.type = Marker.MESH_RESOURCE
        base_210mm_marker.action = Marker.ADD
        base_210mm_marker.mesh_resource = 'package://markers/meshes/base_210mm.stl'
        base_210mm_marker.pose.position.x = 0.0
        base_210mm_marker.pose.position.y = 0.0
        base_210mm_marker.pose.position.z = -0.0205
        q = t3d.euler.euler2quat(1.57, 0.0, 1.57)
        base_210mm_marker.pose.orientation.x = q[1]
        base_210mm_marker.pose.orientation.y = q[2]
        base_210mm_marker.pose.orientation.z = q[3]
        base_210mm_marker.pose.orientation.w = q[0]
        base_210mm_marker.scale.x = 1.0
        base_210mm_marker.scale.y = 1.0
        base_210mm_marker.scale.z = 1.0
        base_210mm_marker.color.r = 0.0
        base_210mm_marker.color.g = 1.0
        base_210mm_marker.color.b = 0.5
        base_210mm_marker.color.a = 1.0
        base_210mm_marker.frame_locked = True

        prop1_marker = Marker()
        prop1_marker.header.frame_id = 'prop_1'
        prop1_marker.header.stamp = self.get_clock().now().to_msg()
        prop1_marker.id = 1
        prop1_marker.type = Marker.MESH_RESOURCE
        prop1_marker.action = Marker.ADD
        prop1_marker.mesh_resource = 'package://markers/meshes/propeller_ccw_puller_5in.stl'
        prop1_marker.pose.position.x = 0.0
        prop1_marker.pose.position.y = 0.0
        prop1_marker.pose.position.z = -0.004
        prop1_marker.pose.orientation.x = 0.0
        prop1_marker.pose.orientation.y = 0.0
        prop1_marker.pose.orientation.z = 0.0
        prop1_marker.pose.orientation.w = 1.0
        prop1_marker.scale.x = 1.0
        prop1_marker.scale.y = 1.0
        prop1_marker.scale.z = 1.0
        prop1_marker.color.r = 0.5
        prop1_marker.color.g = 0.0
        prop1_marker.color.b = 1.0
        prop1_marker.color.a = 1.0
        prop1_marker.frame_locked = True

        prop2_marker = Marker()
        prop2_marker.header.frame_id = 'prop_2'
        prop2_marker.header.stamp = self.get_clock().now().to_msg()
        prop2_marker.id = 2
        prop2_marker.type = Marker.MESH_RESOURCE
        prop2_marker.action = Marker.ADD
        prop2_marker.mesh_resource = 'package://markers/meshes/propeller_cw_puller_5in.stl'
        prop2_marker.pose.position.x = 0.0
        prop2_marker.pose.position.y = 0.0
        prop2_marker.pose.position.z = -0.004
        prop2_marker.pose.orientation.x = 0.0
        prop2_marker.pose.orientation.y = 0.0
        prop2_marker.pose.orientation.z = 0.0
        prop2_marker.pose.orientation.w = 1.0
        prop2_marker.scale.x = 1.0
        prop2_marker.scale.y = 1.0
        prop2_marker.scale.z = 1.0
        prop2_marker.color.r = 0.5
        prop2_marker.color.g = 0.0
        prop2_marker.color.b = 1.0
        prop2_marker.color.a = 1.0
        prop2_marker.frame_locked = True

        prop3_marker = Marker()
        prop3_marker.header.frame_id = 'prop_3'
        prop3_marker.header.stamp = self.get_clock().now().to_msg()
        prop3_marker.id = 3
        prop3_marker.type = Marker.MESH_RESOURCE
        prop3_marker.action = Marker.ADD
        prop3_marker.mesh_resource = 'package://markers/meshes/propeller_ccw_puller_5in.stl'
        prop3_marker.pose.position.x = 0.0
        prop3_marker.pose.position.y = 0.0
        prop3_marker.pose.position.z = -0.004
        prop3_marker.pose.orientation.x = 0.0
        prop3_marker.pose.orientation.y = 0.0
        prop3_marker.pose.orientation.z = 0.0
        prop3_marker.pose.orientation.w = 1.0
        prop3_marker.scale.x = 1.0
        prop3_marker.scale.y = 1.0
        prop3_marker.scale.z = 1.0
        prop3_marker.color.r = 0.5
        prop3_marker.color.g = 0.0
        prop3_marker.color.b = 1.0
        prop3_marker.color.a = 1.0
        prop3_marker.frame_locked = True

        prop4_marker = Marker()
        prop4_marker.header.frame_id = 'prop_4'
        prop4_marker.header.stamp = self.get_clock().now().to_msg()
        prop4_marker.id = 4
        prop4_marker.type = Marker.MESH_RESOURCE
        prop4_marker.action = Marker.ADD
        prop4_marker.mesh_resource = 'package://markers/meshes/propeller_cw_puller_5in.stl'
        prop4_marker.pose.position.x = 0.0
        prop4_marker.pose.position.y = 0.0
        prop4_marker.pose.position.z = -0.004
        prop4_marker.pose.orientation.x = 0.0
        prop4_marker.pose.orientation.y = 0.0
        prop4_marker.pose.orientation.z = 0.0
        prop4_marker.pose.orientation.w = 1.0
        prop4_marker.scale.x = 1.0
        prop4_marker.scale.y = 1.0
        prop4_marker.scale.z = 1.0
        prop4_marker.color.r = 0.5
        prop4_marker.color.g = 0.0
        prop4_marker.color.b = 1.0
        prop4_marker.color.a = 1.0
        prop4_marker.frame_locked = True

        self.marker_publisher.publish(base_210mm_marker)
        self.marker_publisher.publish(prop1_marker)
        self.marker_publisher.publish(prop2_marker)
        self.marker_publisher.publish(prop3_marker)
        self.marker_publisher.publish(prop4_marker)

        self.tf_broadcaster.sendTransform([odom_base_link_transform, odom_base_footprint_transform,
                                           base_link_prop_1_transform, base_link_prop_2_transform,
                                           base_link_prop_3_transform, base_link_prop_4_transform,])

def main():
    rclpy.init()

    node = PuzzleDrone()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
        node.get_logger().info("Shutting down the node...")
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()