import rclpy
import transforms3d as t3d
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        
        self.t = TransformStamped()
        self.t2 = TransformStamped()

        self.tf_br = TransformBroadcaster(self)

        timer_period = 0.1
        self.create_timer(timer_period, self.timer_callback)

        self.start_time = self.get_clock().now()
        self.omega = 0.1

    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'world'
        self.t.child_frame_id = 'moving_robot_3'
        self.t.transform.translation.x = 0.5*np.sin(self.omega*elapsed_time)
        self.t.transform.translation.y = 0.5*np.cos(self.omega*elapsed_time)
        self.t.transform.translation.z = 0.0
        q = t3d.euler.euler2quat(0.0, 0.0, -self.omega*elapsed_time)
        self.t.transform.rotation.x = q[1]
        self.t.transform.rotation.y = q[2]
        self.t.transform.rotation.z = q[3]
        self.t.transform.rotation.w = q[0]

        self.t2.header.stamp = self.get_clock().now().to_msg()
        self.t2.header.frame_id = 'world'
        self.t2.child_frame_id = 'moving_robot_4'
        self.t2.transform.translation.x = 1.0
        self.t2.transform.translation.y = 1.0
        self.t2.transform.translation.z = 1.0
        q2 = t3d.euler.euler2quat(elapsed_time, elapsed_time, 0.0)
        self.t2.transform.rotation.x = q2[1]
        self.t2.transform.rotation.y = q2[2]
        self.t2.transform.rotation.z = q2[3]
        self.t2.transform.rotation.w = q2[0]

        self.tf_br.sendTransform([self.t, self.t2])

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