import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        timer_period = 0.1
        self.create_timer(timer_period, self.timer_callback)

        self.ctrl_joints = JointState()
        self.ctrl_joints.header.stamp = self.get_clock().now().to_msg()
        self.ctrl_joints.name = ['joint_revolute', 'joint_prismatic', 'joint_continuous_1', 'joint_continuous_2']
        self.ctrl_joints.position = [0.0] * len(self.ctrl_joints.name)
        self.ctrl_joints.velocity = [0.0] * len(self.ctrl_joints.name)
        self.ctrl_joints.effort = [0.0] * len(self.ctrl_joints.name)

        self.i = 0.0
        self.sign = 1

    def timer_callback(self):
        time = self.get_clock().now().nanoseconds/1e9
        self.ctrl_joints.header.stamp = self.get_clock().now().to_msg()
        self.ctrl_joints.position[0] = -0.785+2*0.785*self.i
        self.ctrl_joints.position[1] = -2+4*self.i
        self.ctrl_joints.position[2] = 0.5*time
        self.ctrl_joints.position[3] = 0.5*time

        if self.i > 1:
            self.sign = -1
        elif self.i  < 0:
            self.sign = 1
        self.i  = self.i  + self.sign*0.1

        self.publisher.publish(self.ctrl_joints)

def main():
    rclpy.init()
    
    node = FramePublisher()

    try:
        rclpy.spin(node)
    except:
        node.get_logger().info('Node interrupted. Shutting down...')
        if rclpy.ok():
            rclpy.shutdown()
    
    node.destroy_node()

if __name__ == '__main__':
    main()
