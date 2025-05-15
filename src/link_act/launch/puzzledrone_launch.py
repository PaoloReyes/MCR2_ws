import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_filename = 'puzzle_drone.urdf'
    urdf_path = os.path.join(get_package_share_directory('link_act'), 'urdf', urdf_filename)

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    world_to_map_transform_node = Node(name='world_to_map_transform',
                                        package='tf2_ros',
                                        executable='static_transform_publisher',
                                        arguments=['--x', '1', '--y', '0', '--z', '0', 
                                                    '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                    '--frame-id', 'world', '--child-frame-id', 'map'],)
    
    map_to_odom_transform_node = Node(name='map_to_odom_transform',
                                        package='tf2_ros',
                                        executable='static_transform_publisher',
                                        arguments=['--x', '0', '--y', '1', '--z', '0', 
                                                    '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                    '--frame-id', 'map', '--child-frame-id', 'odom'],)
    
    robot_state_publisher_node = Node(name='robot_state_publisher',
                                    package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    output='screen',
                                    parameters=[{'robot_description': robot_description}],)
    
    puzzle_drone_node = Node(name='puzzle_drone',
                            package='link_act',
                            executable='puzzle_drone',
                            output='screen',)
    
    rviz_node = Node(name='rviz2',
                    package='rviz2',
                    executable='rviz2',
                    output='screen',)
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                        package='rqt_tf_tree',
                        executable='rqt_tf_tree',
                        output='screen',)

    return LaunchDescription([world_to_map_transform_node, map_to_odom_transform_node, robot_state_publisher_node, 
                              puzzle_drone_node, rviz_node, rqt_tf_tree_node])