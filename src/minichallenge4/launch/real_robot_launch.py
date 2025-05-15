import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the nodes parameters file
    nodes_params_filename = 'puzzlebot_nodes_params.yaml'
    nodes_params_path = os.path.join(get_package_share_directory('minichallenge4'), 'config', nodes_params_filename)

    # Get the global parameters file
    global_params_filename = 'puzzlebot_global_params.yaml'
    global_params_path = os.path.join(get_package_share_directory('minichallenge4'), 'config', global_params_filename)
    # Read the global parameters file
    with open(global_params_path, 'r') as file:
        global_params = yaml.safe_load(file)

    # Get the robots declared in the config file
    with open(nodes_params_path, 'r') as file:
        nodes_params = yaml.safe_load(file)

    # Iterative puzzlebot nodes
    puzzlebot_nodes = []
    for puzzlebot_namespace in nodes_params.keys():
        puzzlebot_controller_node = Node(name='puzzlebot_controller',
                                        package='minichallenge4',
                                        executable='puzzlebot_controller',
                                        output='screen',
                                        parameters=[nodes_params_path],
                                        namespace=puzzlebot_namespace,
                                        remappings=[(f'/{puzzlebot_namespace}/cmd_vel', '/cmd_vel')],
                                        )
        
        puzzlebot_localization_node = Node(name='puzzlebot_localization',
                                        package='minichallenge4',
                                        executable='puzzlebot_localization',
                                        output='screen',
                                        parameters=[nodes_params_path, global_params],
                                        namespace=puzzlebot_namespace,
                                        remappings=[(f'/{puzzlebot_namespace}/wr', '/VelocityEncR'),
                                                    (f'/{puzzlebot_namespace}/wl', '/VelocityEncL'),],)
        
        puzzlebot_nodes.append(puzzlebot_controller_node)
        puzzlebot_nodes.append(puzzlebot_localization_node)

    # Launch description
    l_d = LaunchDescription([*puzzlebot_nodes,])
    

    return l_d
