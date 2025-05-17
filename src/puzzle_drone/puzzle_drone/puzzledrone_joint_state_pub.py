import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class DronePublisher(Node):

    def __init__(self):
        super().__init__('puzzledorone_joint_pub')

        self.namespace = self.get_namespace().rstrip('/')

        self.declare_parameter('init_pos_x', 0.0)
        self.declare_parameter('init_pos_y', 0.0)
        self.declare_parameter('init_pos_z', 1.0)
        self.declare_parameter('init_pos_yaw', np.pi/2)
        self.declare_parameter('init_pos_pitch', 0.0)
        self.declare_parameter('init_pos_roll', 0.0)
        self.declare_parameter('odom_frame', 'odom')

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value.strip('/')


        #Drone Initial Pose
        self.intial_pos_x = self.get_parameter('init_pos_x').value
        self.intial_pos_y = self.get_parameter('init_pos_y').value
        self.intial_pos_z = self.get_parameter('init_pos_z').value
        self.intial_pos_yaw = self.get_parameter('init_pos_yaw').value
        self.intial_pos_pitch = self.get_parameter('init_pos_pitch').value
        self.intial_pos_roll = self.get_parameter('init_pos_roll').value

        #Angular velocity for the pose change and propellers
        self.omega = 0.5
        self.omega_prop = 100.0

        #Define Transformations
        self.define_TF()

        #initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["prop1_joint", "prop2_joint", "prop3_joint", "prop4_joint"]
        self.ctrlJoints.position = [0.0] * 4
        self.ctrlJoints.velocity = [0.0] * 4
        self.ctrlJoints.effort = [0.0] * 4

        #Create Transform Boradcasters
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_base_footprint = TransformBroadcaster(self)

        #Publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)


    #Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9

        #Create Trasnform Messages
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_link_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw+self.omega*time)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]

        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_footprint_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.intial_pos_yaw+self.omega*time)       
        self.base_footprint_tf.transform.rotation.x = q[1]
        self.base_footprint_tf.transform.rotation.y = q[2]
        self.base_footprint_tf.transform.rotation.z = q[3]
        self.base_footprint_tf.transform.rotation.w = q[0]

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[0] = self.omega_prop*time
        self.ctrlJoints.position[1] = -self.omega_prop*time
        self.ctrlJoints.position[2] = self.omega_prop*time
        self.ctrlJoints.position[3] = -self.omega_prop*time

        self.tf_br_base.sendTransform(self.base_link_tf)
        self.tf_br_base_footprint.sendTransform(self.base_footprint_tf)

        self.publisher.publish(self.ctrlJoints)

    def define_TF(self):

        #Create Trasnform Messages
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.header.frame_id = self.odom_frame
        self.base_footprint_tf.child_frame_id = f'{self.namespace}/base_footprint'
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y
        self.base_footprint_tf.transform.translation.z = 0.0
        q_foot = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_footprint_tf.transform.rotation.x = q_foot[1]
        self.base_footprint_tf.transform.rotation.y = q_foot[2]
        self.base_footprint_tf.transform.rotation.z = q_foot[3]
        self.base_footprint_tf.transform.rotation.w = q_foot[0]


        #Create Trasnform Messages
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.header.frame_id = self.odom_frame
        self.base_link_tf.child_frame_id = f'{self.namespace}/base_link'
        self.base_link_tf.transform.translation.x = self.intial_pos_x
        self.base_link_tf.transform.translation.y = self.intial_pos_y
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]


def main(args=None):
    rclpy.init(args=args)

    node = DronePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()