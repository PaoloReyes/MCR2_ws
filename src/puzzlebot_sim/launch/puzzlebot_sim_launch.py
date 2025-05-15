import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_filename = 'puzzlebot.urdf'
    urdf_path = os.path.join(get_package_share_directory('puzzlebot_sim'), 'urdf', urdf_filename)

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '1', '--y', '1', '--z', '0', 
                                                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)
    
    robot_description_node = Node(name='robot_state_publisher',
                                package='robot_state_publisher',
                                executable='robot_state_publisher',
                                output='screen',
                                parameters=[{'robot_description': robot_description}],)
    
    puzzlebot_joint_publisher_node = Node(name='puzzlebot_joint_publisher',
                                        package='puzzlebot_sim',
                                        executable='puzzlebot_joint_publisher',
                                        output='screen',)
    
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                        package='rqt_tf_tree',
                        executable='rqt_tf_tree',
                        output='screen',)
    
    rviz_node = Node(name='rviz2',
                    package='rviz2',
                    executable='rviz2',
                    output='screen',)

    l_d = LaunchDescription([map_odom_transform_node, robot_description_node, puzzlebot_joint_publisher_node, rqt_tf_tree_node, rviz_node])

    return l_d
