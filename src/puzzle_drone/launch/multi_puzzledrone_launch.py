import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file_name = 'puzzle_drone.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('puzzle_drone'),
        'urdf',
        urdf_file_name)
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    puzzledrone_node = Node(name="puzzledrone_joint_pub",
                            package='puzzle_drone',
                            executable='puzzledrone_joint_state_pub',
                            parameters=[{
                                'init_pos_x': 0.0,
                                'init_pos_y': 0.0,
                                'init_pos_z': 1.0,
                                'init_pos_roll': 0.0,
                                'init_pos_pitch': 0.0,
                                'init_pos_yaw': 1.57,
                            }],
                            namespace='group1',
                            )
    
    puzzledrone_node_2 = Node(name="puzzledrone_joint_pub",
                            package='puzzle_drone',
                            executable='puzzledrone_joint_state_pub',
                            parameters=[{
                                'init_pos_x': 1.0,
                                'init_pos_y': 0.0,
                                'init_pos_z': 1.0,
                                'init_pos_roll': 0.0,
                                'init_pos_pitch': 0.0,
                                'init_pos_yaw': 1.57,
                            }],
                            namespace='group2',
                            )
    
    robot_state_pub_node = Node(name="robot_state_publisher",
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            parameters=[{'frame_prefix': 'group1/',
                                        'robot_description': robot_desc}],
                            namespace='group1',
                            )
    
    robot_state_pub_node_2 = Node(name="robot_state_publisher",
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            parameters=[{'frame_prefix': 'group2/',
                                        'robot_description': robot_desc}],
                            namespace='group2',
                            )
    
    return LaunchDescription([puzzledrone_node, 
                              puzzledrone_node_2,
                              robot_state_pub_node,
                              robot_state_pub_node_2])