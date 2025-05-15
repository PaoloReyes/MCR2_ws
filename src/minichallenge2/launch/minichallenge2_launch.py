import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the URDF file
    urdf_filename = 'puzzlebot.urdf'
    urdf_path = os.path.join(get_package_share_directory('minichallenge2'), 'urdf', urdf_filename)
    # Read the URDF file
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # Get the nodes parameters file
    nodes_params_filename = 'puzzlebot_nodes_params.yaml'
    nodes_params_path = os.path.join(get_package_share_directory('minichallenge2'), 'config', nodes_params_filename)

    # Get the global parameters file
    global_params_filename = 'puzzlebot_global_params.yaml'
    global_params_path = os.path.join(get_package_share_directory('minichallenge2'), 'config', global_params_filename)
    # Read the global parameters file
    with open(global_params_path, 'r') as file:
        global_params = yaml.safe_load(file)

    # Transform from map to odom
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '1', '--y', '1', '--z', '0', 
                                                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)
    
    # Puzzlebot trajectory generator node
    puzzlebot_trajectory_generator_node = Node(name='puzzlebot_trajectory_generator',
                                            package='minichallenge2',
                                            executable='puzzlebot_trajectory_generator',
                                            output='screen',
                                            parameters=[nodes_params_path],)

    # Puzzlebot controller node
    puzzlebot_controller_node = Node(name='puzzlebot_controller',
                                    package='minichallenge2',
                                    executable='puzzlebot_controller',
                                    output='screen',
                                    parameters=[nodes_params_path],)
    
    # Puzzlebot simulation node
    puzzlebot_sim_node = Node(name='puzzlebot_sim',
                            package='minichallenge2',
                            executable='puzzlebot_sim',
                            output='screen',
                            parameters=[nodes_params_path, global_params],)
    
    # Puzzlebot localization node
    puzzlebot_localization_node = Node(name='puzzlebot_localization',
                                package='minichallenge2',
                                executable='puzzlebot_localization',
                                output='screen',
                                parameters=[nodes_params_path, global_params],)
    
    # Puzzlebot joint publisher node
    puzzlebot_joint_publisher_node = Node(name='puzzlebot_joint_publisher',
                                        package='minichallenge2',
                                        executable='puzzlebot_joint_publisher',
                                        output='screen',
                                        parameters=[global_params],)
    
    # Robot state publisher node
    robot_description_node = Node(name='robot_state_publisher',
                                package='robot_state_publisher',
                                executable='robot_state_publisher',
                                output='screen',
                                parameters=[{'robot_description': robot_description}],)
    
    # RQT nodes
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                        package='rqt_tf_tree',
                        executable='rqt_tf_tree',
                        output='screen',)
    
    rqt_graph_node = Node(name='rqt_graph',
                    package='rqt_graph',
                    executable='rqt_graph',
                    output='screen',)
    
    # RViz node
    rviz_node = Node(name='rviz2',
                    package='rviz2',
                    executable='rviz2',
                    output='screen',)

    # Launch description
    l_d = LaunchDescription([map_odom_transform_node,
                            puzzlebot_trajectory_generator_node,
                            puzzlebot_controller_node,
                            puzzlebot_sim_node,
                            puzzlebot_localization_node,
                            puzzlebot_joint_publisher_node,
                            robot_description_node,
                            rqt_tf_tree_node,
                            rqt_graph_node,
                            rviz_node,])
    

    return l_d
