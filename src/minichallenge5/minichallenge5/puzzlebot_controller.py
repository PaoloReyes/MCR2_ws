import numpy as np
import rclpy
import transforms3d as t3d

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class PuzzlebotController(Node):
    def __init__(self):
        super().__init__('puzzlebot_controller_node')

        # Declare parameters
        self.declare_parameter('controller_update_rate', 25.0)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('angular_tolerance', 0.1)
        self.declare_parameter('angular_adjustment', 0.1)
        self.declare_parameter('following_walls_distance', 0.1)
        self.declare_parameter('front_stop_distance', 0.2)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('p2p_v_Kp', 0.5)
        self.declare_parameter('p2p_w_Kp', 0.5)
        self.declare_parameter('fw_w_Kp', 0.1)
        self.declare_parameter('fw_e_Kp', 4.0)
        self.declare_parameter('fw_linear_speed', 0.1)
        self.declare_parameter('fw_outer_corner_angular_speed', 0.1)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('w_max', 3.14)
        self.declare_parameter('side_open_angle', np.pi/6)
        self.declare_parameter('front_open_angle', np.pi/3)
        self.declare_parameter('target_open_angle', np.pi/3)

        # Subscribers
        self.create_subscription(Pose2D, 'setpoint', self.setpoint_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.next_point_publisher = self.create_publisher(Bool, 'goal_reached', 10)

        # Node variables
        self.controller_timer = None
        self.update_rate = self.get_parameter('controller_update_rate').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value
        self.angular_adjustment = self.get_parameter('angular_adjustment').get_parameter_value().double_value
        self.following_walls_distance = self.get_parameter('following_walls_distance').get_parameter_value().double_value
        self.front_stop_distance = self.get_parameter('front_stop_distance').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.p2p_v_Kp = self.get_parameter('p2p_v_Kp').get_parameter_value().double_value
        self.p2p_w_Kp = self.get_parameter('p2p_w_Kp').get_parameter_value().double_value
        self.fw_w_Kp = self.get_parameter('fw_w_Kp').get_parameter_value().double_value
        self.fw_e_Kp = self.get_parameter('fw_e_Kp').get_parameter_value().double_value
        self.fw_linear_speed = self.get_parameter('fw_linear_speed').get_parameter_value().double_value
        self.fw_outer_corner_angular_speed = self.get_parameter('fw_outer_corner_angular_speed').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.w_max = self.get_parameter('w_max').get_parameter_value().double_value
        self.side_open_angle = self.get_parameter('side_open_angle').get_parameter_value().double_value
        self.front_open_angle = self.get_parameter('front_open_angle').get_parameter_value().double_value
        self.target_open_angle = self.get_parameter('target_open_angle').get_parameter_value().double_value
        self.robot_pose = Pose2D()
        self.robot_setpoint = Pose2D()
        self.closest_object_angle = 0.0
        self.controller_mode = 'p2p_controller'
        self.following_walls_directions = {'left': 1, 'right': -1}
        self.following_walls_direction = 'left'
        self.min_side_region_distance = 0.0
        self.min_front_region_distance = 0.0
        self.min_back_side_region_distance = 0.0
        self.min_back_side_outside_region_distance = 0.0

        # Log the node start
        self.get_logger().info('Puzzlebot Controller Node has been started.')

    def setpoint_callback(self, msg):
        # If the controller timer is running, cancel it
        if self.controller_timer is not None:
            self.controller_timer.cancel()

        # Start the controller timer with the new update rate   
        self.controller_timer = self.create_timer(1.0/self.update_rate, self.controller_callback)
        # Update the robot pose with the new setpoint
        self.robot_setpoint = msg

    def odom_callback(self, msg):
        # Extract [x, y] position from the odometry message
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        # Extract quaternion orientation from the odometry message
        q = msg.pose.pose.orientation
        # Convert quaternion to Euler angles and update the robot pose
        _, _, self.robot_pose.theta = t3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    def lidar_callback(self, msg):
        # Get the lidar readings
        lidar_readings = np.array(msg.ranges)

        # Get the closest object distance
        closest_object_distance = np.min(lidar_readings)
        # Get the closest object angle
        self.closest_object_angle = msg.angle_min + np.argmin(lidar_readings)*msg.angle_increment
        # Clamp the closest object angle to be within [-pi, pi]
        self.closest_object_angle = np.atan2(np.sin(self.closest_object_angle), np.cos(self.closest_object_angle))

        # Calculate the angle error to face the target
        theta_to_face_target_error = self._get_angle_error_to_face_target(self.robot_pose, self.robot_setpoint)
        # Get the front region data
        start_angle = theta_to_face_target_error - msg.angle_min - self.target_open_angle
        end_angle = theta_to_face_target_error - msg.angle_min + self.target_open_angle
        # Clamp the angles to be within [-pi, pi]
        start_angle = np.atan2(np.sin(start_angle), np.cos(start_angle))
        end_angle = np.atan2(np.sin(end_angle), np.cos(end_angle))
        # Move the angles to the range of [0, 2*pi]
        start_angle = self._symmetric_theta_shift(start_angle)
        end_angle = self._symmetric_theta_shift(end_angle)
        # Get the start and end indices for the front region
        start_index, end_index = self._get_index_from_angle(start_angle, end_angle, msg.angle_increment)
        # Get the front region closest object distance
        min_target_region_distance = self._get_min_distance_from_region(lidar_readings, start_index, end_index, msg.range_min)

        # Get the side region start and end angles
        start_angle, end_angle = self._get_sides_start_end_angles(self.following_walls_direction, msg.angle_min, self.side_open_angle, self.side_open_angle)
        # Get the start and end indices for the region
        start_index, end_index = self._get_index_from_angle(start_angle, end_angle, msg.angle_increment)
        # Get the side region closest object distance
        self.min_side_region_distance = self._get_min_distance_from_region(lidar_readings, start_index, end_index, msg.range_min)

        # Get the front region data
        start_angle = -msg.angle_min - self.front_open_angle
        end_angle = -msg.angle_min + self.front_open_angle
        # Clamp the angles to be within [-pi, pi]
        start_angle = np.atan2(np.sin(start_angle), np.cos(start_angle))
        end_angle = np.atan2(np.sin(end_angle), np.cos(end_angle))
        # Move the angles to the range of [0, 2*pi]
        start_angle = self._symmetric_theta_shift(start_angle)
        end_angle = self._symmetric_theta_shift(end_angle)
        # Get the start and end indices for the front region
        start_index, end_index = self._get_index_from_angle(start_angle, end_angle, msg.angle_increment)
        # Get the front region closest object distance
        self.min_front_region_distance = self._get_min_distance_from_region(lidar_readings, start_index, end_index, msg.range_min)

        # Get the back side region start and end angles
        start_angle, end_angle = self._get_sides_start_end_angles(self.following_walls_direction, msg.angle_min, 0, self.side_open_angle)
        # Get the start and end indices for the region
        start_index, end_index = self._get_index_from_angle(start_angle, end_angle, msg.angle_increment)
        # Get the back side region closest object distance
        self.min_back_side_region_distance = self._get_min_distance_from_region(lidar_readings, start_index, end_index, msg.range_min)

        # Get the start and end indices for the region with hysteresis
        start_index, end_index = self._get_index_from_angle(start_angle, end_angle, msg.angle_increment)
        # Get the min outside back region distance
        self.min_back_side_outside_region_distance = self._get_min_distance_outside_region(lidar_readings, start_index, end_index, msg.range_min)

        # Determine the controller mode 
        if self.min_front_region_distance < self.following_walls_distance*1.5 and self.controller_mode == 'p2p_controller':
            self.following_walls_direction = np.random.choice(list(self.following_walls_directions.keys()))
            self.controller_mode = 'following_walls'
        elif min_target_region_distance > self._get_euclidian_distance_between_poses(self.robot_pose, self.robot_setpoint) and self.controller_mode == 'following_walls':
            self.controller_mode = 'p2p_controller'

    def controller_callback(self):
        # Calculate the distance to the setpoint
        distance_to_target = self._get_euclidian_distance_between_poses(self.robot_pose, self.robot_setpoint)
        
        # Create a Twist message to publish the robot velocity
        twist_msg = Twist()
        if distance_to_target < self.distance_tolerance and abs(self.robot_setpoint.theta-self.robot_pose.theta) < self.angular_tolerance:
            self.next_point_publisher.publish(Bool(data=True))
            self.controller_timer.cancel()
        else:
            if self.controller_mode == 'following_walls':
                # Get the angle to separate the robot from the wall faster
                angle_to_separate = self.closest_object_angle + np.pi
                # Clamp the angle to separate to be within [-pi, pi]
                angle_to_separate = np.atan2(np.sin(angle_to_separate), np.cos(angle_to_separate))

                # Calculate the error to the distance to the wall
                distance_to_wall_error = self.min_side_region_distance - self.following_walls_distance

                # Calculate the tangent angle to the wall
                if self.following_walls_direction == 'left':
                    tangent_angle = angle_to_separate + np.pi/2
                elif self.following_walls_direction == 'right':
                    tangent_angle = angle_to_separate - np.pi/2
                # Clamp the tangent angle to be within [-pi, pi]
                tangent_angle = np.atan2(np.sin(tangent_angle), np.cos(tangent_angle))

                # Perform wall-following control
                # Calculate the linear speed
                linear_speed = self.fw_linear_speed
                if self.min_front_region_distance < self.front_stop_distance:
                    linear_speed = 0
                elif self.min_front_region_distance < 2*self.front_stop_distance:
                    linear_speed /= 2
                # Calculate the angular speed
                angular_speed = self.fw_w_Kp * tangent_angle +\
                                self.following_walls_directions[self.following_walls_direction] * self.fw_e_Kp * distance_to_wall_error
                
                # Perform a outer corner control algorithm if exists
                if self.min_back_side_region_distance < self.lookahead_distance and self.min_back_side_outside_region_distance > self.lookahead_distance:
                    linear_speed = self.fw_linear_speed
                    angular_speed = self.fw_outer_corner_angular_speed
                    if self.following_walls_direction == 'right':
                        angular_speed = -angular_speed

                # Constrain speeds
                linear_speed = np.clip(linear_speed, -self.v_max, self.v_max)
                angular_speed = np.clip(angular_speed, -self.w_max, self.w_max)

                # Update the Twist message with the calculated velocities
                twist_msg.linear.x = linear_speed
                twist_msg.angular.z = angular_speed
            elif self.controller_mode == 'p2p_controller':
                # Calculate the angle error to face the target
                theta_to_face_target_error = self._get_angle_error_to_face_target(self.robot_pose, self.robot_setpoint)

                # Perform orientation control on the move
                linear_speed = self.p2p_v_Kp * distance_to_target
                angular_speed = self.p2p_w_Kp * theta_to_face_target_error

                # Perform a first rotate then move control
                if abs(theta_to_face_target_error) > self.angular_adjustment:
                    linear_speed = 0

                # Constrain speeds
                linear_speed = np.clip(linear_speed, -self.v_max, self.v_max)
                angular_speed = np.clip(angular_speed, -self.w_max, self.w_max)
                
                # Update the Twist message with the calculated velocities
                twist_msg.linear.x = linear_speed
                twist_msg.angular.z = angular_speed

        # Publish the Twist message
        self.cmd_vel_publisher.publish(twist_msg)

    def _get_euclidian_distance_between_poses(self, pose1, pose2):
        # Calculate the Euclidean distance between two poses with sign
        return np.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
    
    def _symmetric_theta_shift(self, theta):
        if theta >= 0:
            return theta
        else:
            return 2 * np.pi + theta
        
    def _get_index_from_angle(self, start_angle, end_angle, angle_increment):
        # Clamp the angles to be within [-pi, pi]
        start_angle = np.atan2(np.sin(start_angle), np.cos(start_angle))
        end_angle = np.atan2(np.sin(end_angle), np.cos(end_angle))
        # Move the angles to the range of [0, 2*pi]
        start_angle = self._symmetric_theta_shift(start_angle)
        end_angle = self._symmetric_theta_shift(end_angle)
        # Get the start and end indices for the left region
        start_index = int(start_angle / angle_increment)
        end_index = int(end_angle / angle_increment)

        # Return the start and end indices
        return start_index, end_index
    
    def _get_min_distance_from_region(self, lidar_readings, start_index, end_index, range_min):
        # Check if overflow occurs and create a valid readings list
        if start_index > end_index:
            right_readings = lidar_readings[start_index:]
            left_readings = lidar_readings[:end_index]
            valid_readings = np.concatenate((right_readings, left_readings))
        else:
            valid_readings = lidar_readings[start_index:end_index]

        # Return the minimum distance from the valid readings
        return max(np.min(valid_readings), range_min)
    
    def _get_sides_start_end_angles(self, direction, angle_min, front_width, back_width):
        # Get the side region start and end angles
        if direction == 'left':
            start_angle = np.pi/2 - angle_min - front_width
            end_angle = np.pi/2 - angle_min + back_width
        elif direction == 'right':
            start_angle = 3*np.pi/2 - angle_min - back_width
            end_angle = 3*np.pi/2 - angle_min + front_width
        else:
            raise ValueError("Direction must be 'left' or 'right'.")

        return start_angle, end_angle

    def _get_min_distance_outside_region(self, lidar_readings, start_index, end_index, range_min):
        # Check if overflow occurs and create a valid readings list
        if start_index > end_index:
            valid_readings = lidar_readings[end_index:start_index]
        else:
            right_readings = lidar_readings[:start_index]
            left_readings = lidar_readings[end_index:]
            valid_readings = np.concatenate((right_readings, left_readings))

        # Return the minimum distance from the valid readings
        return max(np.min(valid_readings), range_min)
    
    def _get_angle_error_to_face_target(self, robot_pose, target_pose):
        # Calculate the angle between the robot and the setpoint
        angle_between_poses = np.atan2(self.robot_setpoint.y - self.robot_pose.y, self.robot_setpoint.x - self.robot_pose.x)
        # Calculate the angle to face the target and the setpoint error
        theta_to_face_target_error = angle_between_poses - self.robot_pose.theta
        # Return the normalized angle within [-pi, pi]
        return np.atan2(np.sin(theta_to_face_target_error), np.cos(theta_to_face_target_error))

def main():
    rclpy.init()
    node = PuzzlebotController()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().error(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()