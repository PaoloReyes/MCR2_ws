import os
import transforms3d as t3d
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch arguments declaration
    launch_rqt_arg = DeclareLaunchArgument('rqt', default_value='false', description='Flag to launch RQT nodes')
    launch_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')

    # Read launch argument
    launch_rqt = LaunchConfiguration('rqt')
    launch_use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the package share directory
    package_share_dir = get_package_share_directory('minichallenge5')

    # Node variables
    urdf_filename = 'puzzlebot.xacro'
    nodes_params_filename = 'puzzlebot_nodes_params.yaml'
    global_params_filename = 'puzzlebot_global_params.yaml'
    rviz_config_filename = 'puzzlebot_view.rviz'
    world_filename = 'puzzlebot_world.world'
    ros_gz_bridge_config_filename = 'puzzlebot_bridge.yaml'

    # Get the path to important files
    urdf_path = os.path.join(package_share_dir, 'urdf', urdf_filename)
    nodes_params_path = os.path.join(package_share_dir, 'config', nodes_params_filename)
    rviz_config_path = os.path.join(package_share_dir, 'rviz', rviz_config_filename)
    global_params_path = os.path.join(package_share_dir, 'config', global_params_filename)
    world_path = os.path.join(package_share_dir, 'worlds', world_filename)
    ros_gz_bridge_config_path = os.path.join(package_share_dir, 'config', ros_gz_bridge_config_filename)

    # Read the global parameters file
    with open(global_params_path, 'r') as file:
        global_params = yaml.safe_load(file)

    # Get the robots declared in the config file
    with open(nodes_params_path, 'r') as file:
        nodes_params = yaml.safe_load(file)

    # Map to odom transform node
    map_odom_transform_node = Node(name='map_odom_transform',
                                    package='tf2_ros',
                                    executable='static_transform_publisher',
                                    output='screen',
                                    arguments=['--x', '0', '--y', '0', '--z', '0', 
                                                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                                '--frame-id', 'map', '--child-frame-id', 'odom'],)

    # Iterative puzzlebot nodes
    puzzlebot_nodes = []
    for puzzlebot_namespace in nodes_params.keys():
        # Read the parameters for each puzzlebot
        initial_x = nodes_params[puzzlebot_namespace]['puzzlebot_localization']['ros__parameters']['initial_pose']['x']
        initial_y = nodes_params[puzzlebot_namespace]['puzzlebot_localization']['ros__parameters']['initial_pose']['y']
        initial_theta = nodes_params[puzzlebot_namespace]['puzzlebot_localization']['ros__parameters']['initial_pose']['theta']

        # Get the robot name and description topic
        robot_name = puzzlebot_namespace
        robot_description_topic = f"/{robot_name}/robot_description"

        # Get the robot description from the URDF file
        robot_description = Command(['xacro ', str(urdf_path),
                                    ' camera_frame:=', 'camera_link_optical',
                                    ' lidar_frame:=', 'laser_frame',
                                    ' prefix:=', f'{puzzlebot_namespace}/',])

        puzzlebot_controller_node = Node(name='puzzlebot_controller',
                                        package='minichallenge5',
                                        executable='puzzlebot_controller',
                                        output='screen',
                                        parameters=[nodes_params_path, {'use_sim_time': launch_use_sim_time}],
                                        namespace=puzzlebot_namespace,)
        
        puzzlebot_localization_node = Node(name='puzzlebot_localization',
                                        package='minichallenge5',
                                        executable='puzzlebot_localization',
                                        output='screen',
                                        parameters=[nodes_params_path, global_params, {'use_sim_time': launch_use_sim_time}],
                                        namespace=puzzlebot_namespace,)
        
        puzzlebot_joint_state_publisher_node = Node(name='puzzlebot_joint_publisher',
                                                package='minichallenge5',
                                                executable='puzzlebot_joint_state_publisher',
                                                output='screen',
                                                parameters=[global_params, {'use_sim_time': launch_use_sim_time}],
                                                namespace=puzzlebot_namespace,)
        
        puzzlebot_sim_node = Node(name='puzzlebot_sim',
                                    package='minichallenge5',
                                    executable='puzzlebot_sim',
                                    output='screen',
                                    parameters=[global_params, {'use_sim_time': launch_use_sim_time}],
                                    namespace=puzzlebot_namespace,)
        
        robot_description_node = Node(name='robot_state_publisher',
                                    package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    output='screen',
                                    parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                                                 'use_sim_time': launch_use_sim_time}],
                                    namespace=puzzlebot_namespace,)

        gz_spawn_puzzlebot_node = Node(package="ros_gz_sim",
                                        executable="create",
                                        arguments=[
                                            "-name", robot_name,
                                            "-topic", robot_description_topic,
                                            "-x", str(initial_x), "-y", str(initial_y), "-Y", str(initial_theta),
                                        ],
                                        output="screen",)
        
        puzzlebot_nodes.append(puzzlebot_controller_node)
        puzzlebot_nodes.append(puzzlebot_localization_node)
        puzzlebot_nodes.append(puzzlebot_joint_state_publisher_node)
        puzzlebot_nodes.append(puzzlebot_sim_node)
        puzzlebot_nodes.append(robot_description_node)
        puzzlebot_nodes.append(gz_spawn_puzzlebot_node)

    # Gz world and puzzlebot launch
    gz_process = ExecuteProcess(cmd=['gz', 'sim', world_path, '-r'],
                                output='screen',)

    # Gz camera process
    q = t3d.euler.euler2quat(0.0, 1.5708, 1.5708)
    gz_camera_process = ExecuteProcess(cmd=['gz', 'service', '-s', '/gui/move_to/pose', 
                                            '--reqtype', 'gz.msgs.GUICamera',
                                            '--reptype', 'gz.msgs.Boolean',
                                            '--timeout', '10000',
                                            '--req', f'pose: {{position: {{x: 0.0, y: 0.0, z: 3.0}} orientation: {{x: {q[1]}, y: {q[2]}, z: {q[3]}, w: {q[0]}}}}}'],
                                        output='screen',)
    
    with open(ros_gz_bridge_config_path, 'r') as file:
        ros_gz_bridge_config = yaml.safe_load(file)

    bridges = []
    for robot_namespace in nodes_params.keys():
        for bridge in ros_gz_bridge_config:
            if bridge['ros_topic_name'] != 'clock' and bridge['ros_topic_name'] != 'cmd_vel':
                bridges.append(f'{robot_namespace}/{bridge["ros_topic_name"]}@{bridge["ros_type_name"]}[{bridge["gz_type_name"]}')
            elif bridge['ros_topic_name'] == 'cmd_vel':
                bridges.append(f'{robot_namespace}/{bridge["ros_topic_name"]}@{bridge["ros_type_name"]}]{bridge["gz_type_name"]}')
            elif bridge['ros_topic_name'] == 'clock':
                bridges.append(f'{bridge["ros_topic_name"]}@{bridge["ros_type_name"]}[{bridge["gz_type_name"]}')
    
    # Bridge ROS topics and Gazebo messages for establishing communication
    gz_bridge_node = Node(package='ros_gz_bridge',
                            executable='parameter_bridge',
                            arguments=bridges,
                            output='screen')
        
    # RQT nodes
    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                        package='rqt_tf_tree',
                        executable='rqt_tf_tree',
                        output='screen',
                        condition=IfCondition(launch_rqt))
    
    rqt_graph_node = Node(name='rqt_graph',
                    package='rqt_graph',
                    executable='rqt_graph',
                    output='screen',
                    condition=IfCondition(launch_rqt))
    
    # RViz node
    rviz_node = Node(name='rviz2',
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path],)

    # Launch description
    l_d = LaunchDescription([launch_rqt_arg,
                            launch_use_sim_time_arg,
                            map_odom_transform_node,
                            gz_process,
                            gz_camera_process,
                            gz_bridge_node,
                            *puzzlebot_nodes,
                            rqt_tf_tree_node,
                            rqt_graph_node,
                            rviz_node,])
    
    return l_d
