import rclpy
import transforms3d as t3d

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        static_br1 = StaticTransformBroadcaster(self)
        static_br2 = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot_3'
        t.transform.translation.x = -2.0
        t.transform.translation.y = -1.0
        t.transform.translation.z = 0.0
        q = t3d.euler.euler2quat(0.0, 0.0, 0.0)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'robot_2'
        t2.child_frame_id = 'robot_4'
        t2.transform.translation.x = 1.0
        t2.transform.translation.y = 1.0
        t2.transform.translation.z = 1.0
        q2 = t3d.euler.euler2quat(1.57, 1.57, 0.0)
        t2.transform.rotation.x = q2[1]
        t2.transform.rotation.y = q2[2]
        t2.transform.rotation.z = q2[3]
        t2.transform.rotation.w = q2[0]

        static_br1.sendTransform(t)
        static_br2.sendTransform(t2)

def main():
    rclpy.init()

    node = FramePublisher()

    try:
        rclpy.spin(node)
    except:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
        
if __name__ == '__main__':
    main()