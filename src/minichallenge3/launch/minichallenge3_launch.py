import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the URDF file
    urdf_filename = 'puzzlebot.urdf'
    urdf_path = os.path.join(get_package_share_directory('minichallenge3'), 'urdf', urdf_filename)
    # Read the URDF file
    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    # Get the nodes parameters file
    nodes_params_filename = 'puzzlebot_nodes_params.yaml'
    nodes_params_path = os.path.join(get_package_share_directory('minichallenge3'), 'config', nodes_params_filename)

    # Get the global parameters file
    global_params_filename = 'puzzlebot_global_params.yaml'
    global_params_path = os.path.join(get_package_share_directory('minichallenge3'), 'config', global_params_filename)
    # Read the global parameters file
    with open(global_params_path, 'r') as file:
        global_params = yaml.safe_load(file)

    # Namespaces for the nodes
    robot1_namespace = 'robot1'
    robot2_namespace = 'robot2'

    # Get the rviz config file
    rviz_config_filename = 'multiple_robots_config.rviz'
    rviz_config_path = os.path.join(get_package_share_directory('minichallenge3'), 'rviz', rviz_config_filename)

    # Transform from map to odom
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '1', '--y', '1', '--z', '0', 
                                                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)
    
    # Puzzlebot trajectory generator nodes
    puzzlebot_trajectory_generator_node_1 = Node(name='puzzlebot_trajectory_generator',
                                                package='minichallenge3',
                                                executable='puzzlebot_trajectory_generator',
                                                output='screen',
                                                parameters=[nodes_params_path],
                                                namespace=robot1_namespace,)
    
    puzzlebot_trajectory_generator_node_2 = Node(name='puzzlebot_trajectory_generator',
                                                package='minichallenge3',
                                                executable='puzzlebot_trajectory_generator',
                                                output='screen',
                                                parameters=[nodes_params_path],
                                                namespace=robot2_namespace,)

    # Puzzlebot controller nodes
    puzzlebot_controller_node_1 = Node(name='puzzlebot_controller',
                                        package='minichallenge3',
                                        executable='puzzlebot_controller',
                                        output='screen',
                                        parameters=[nodes_params_path],
                                        namespace=robot1_namespace,)
    
    puzzlebot_controller_node_2 = Node(name='puzzlebot_controller',
                                        package='minichallenge3',
                                        executable='puzzlebot_controller',
                                        output='screen',
                                        parameters=[nodes_params_path],
                                        namespace=robot2_namespace,)
    
    # Puzzlebot simulation nodes
    puzzlebot_sim_node_1 = Node(name='puzzlebot_sim',
                                package='minichallenge3',
                                executable='puzzlebot_sim',
                                output='screen',
                                parameters=[nodes_params_path, global_params],
                                namespace=robot1_namespace,)
    
    puzzlebot_sim_node_2 = Node(name='puzzlebot_sim',
                                package='minichallenge3',
                                executable='puzzlebot_sim',
                                output='screen',
                                parameters=[nodes_params_path, global_params],
                                namespace=robot2_namespace,)
    
    # Puzzlebot localization nodes
    puzzlebot_localization_node_1 = Node(name='puzzlebot_localization',
                                        package='minichallenge3',
                                        executable='puzzlebot_localization',
                                        output='screen',
                                        parameters=[nodes_params_path, global_params],
                                        namespace=robot1_namespace,)
    
    puzzlebot_localization_node_2 = Node(name='puzzlebot_localization',
                                        package='minichallenge3',
                                        executable='puzzlebot_localization',
                                        output='screen',
                                        parameters=[nodes_params_path, global_params],
                                        namespace=robot2_namespace,)
    
    # Puzzlebot joint publisher nodes
    puzzlebot_joint_publisher_node_1 = Node(name='puzzlebot_joint_publisher',
                                            package='minichallenge3',
                                            executable='puzzlebot_joint_publisher',
                                            output='screen',
                                            parameters=[global_params],
                                            namespace=robot1_namespace,)
    
    puzzlebot_joint_publisher_node_2 = Node(name='puzzlebot_joint_publisher',
                                            package='minichallenge3',
                                            executable='puzzlebot_joint_publisher',
                                            output='screen',
                                            parameters=[global_params],
                                            namespace=robot2_namespace,)

    # Robot state publisher nodes
    robot_description_node_1 = Node(name='robot_state_publisher',
                                    package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    output='screen',
                                    parameters=[{'frame_prefix': f'{robot1_namespace}/',
                                                'robot_description': robot_description}],
                                    namespace=robot1_namespace,)

    robot_description_node_2 = Node(name='robot_state_publisher',
                                    package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    output='screen',
                                    parameters=[{'frame_prefix': f'{robot2_namespace}/',
                                                'robot_description': robot_description}],
                                    namespace=robot2_namespace,)
    
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
                    output='screen',
                    arguments=['-d', rviz_config_path],)

    # Launch description
    l_d = LaunchDescription([map_odom_transform_node,
                            puzzlebot_trajectory_generator_node_1,
                            puzzlebot_trajectory_generator_node_2,
                            puzzlebot_controller_node_1,
                            puzzlebot_controller_node_2,
                            puzzlebot_sim_node_1,
                            puzzlebot_sim_node_2,
                            puzzlebot_localization_node_1,
                            puzzlebot_localization_node_2,
                            puzzlebot_joint_publisher_node_1,
                            puzzlebot_joint_publisher_node_2,
                            robot_description_node_1,
                            robot_description_node_2,
                            rqt_tf_tree_node,
                            rqt_graph_node,
                            rviz_node,])
    

    return l_d
