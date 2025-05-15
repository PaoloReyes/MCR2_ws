import os
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool

class PuzzlebotTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('puzzlebot_trajectory_generator')

        # Internal node variables
        config_path = os.path.join(
            get_package_share_directory('minichallenge2'),
            'config',
            'puzzlebot_trajectories.yaml'
        )

        # Trejectory parameters declaration
        self.declare_parameter('selected_trajectory', 'square')
        self.declare_parameter('trajectories', [0.0, 0.0, 0.0])

        # Get the parameters
        self.selected_trajectory = self.get_parameter('selected_trajectory').get_parameter_value().string_value

        # Subscribers
        self.create_subscription(Bool, '/next_point', self.next_point_callback, 10)
        
        # Publishers
        self.setpoint_publisher = self.create_publisher(Pose2D, '/setpoint', 10)

        # Node variables
        with open(config_path, 'r') as file:
            self.trajectories = yaml.safe_load(file)

        self.trajectory_point_index = 0
        self.last_trajectory = self.selected_trajectory

        # Log the node start
        self.get_logger().info('Puzzlebot trajectory generator node started.')

    def next_point_callback(self, msg):
        # Get the selected trajectory from the parameter server
        self.selected_trajectory = self.get_parameter('selected_trajectory').get_parameter_value().string_value
        # Check if the selected trajectory has changed
        if self.selected_trajectory != self.last_trajectory:
            # Reset the trajectory point index and store the last trajectory
            self.trajectory_point_index = 0
            self.last_trajectory = self.selected_trajectory
    
        # Check if the msg.data is True and if the trajectory point index is less than the length of the trajectory
        if msg.data:
            if self.trajectory_point_index < len(self.trajectories[self.selected_trajectory]):
                # Create a new Pose2D message
                setpoint = Pose2D()
                setpoint.x = self.trajectories[self.selected_trajectory][self.trajectory_point_index][0]
                setpoint.y = self.trajectories[self.selected_trajectory][self.trajectory_point_index][1]
                setpoint.theta = self.trajectories[self.selected_trajectory][self.trajectory_point_index][2]

                # Publish the setpoint
                self.setpoint_publisher.publish(setpoint)
                
                self.trajectory_point_index += 1
            else:
                # If the trajectory is completed, reset the trajectory point index
                self.trajectory_point_index = 0

def main():
    rclpy.init()

    node = PuzzlebotTrajectoryGenerator()

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