from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_transform_node = Node(name='static_tf_1',
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments=['--x', '2', '--y', '1', '--z', '0', 
                                        '--yaw', '1.57', '--pitch', '0', '--roll', '0', 
                                        '--frame-id', 'world', '--child-frame-id', 'robot_1'],)
    
    static_transform_node_2 = Node(name='static_tf_2',
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments=['--x', '1', '--y', '2', '--z', '0', 
                                        '--yaw', '0', '--pitch', '0', '--roll', '0', 
                                        '--frame-id', 'robot_1', '--child-frame-id', 'robot_2'],)
    
    static_transform_node_3 = Node(name='static_tf_3',
                                package='tf_examples',
                                executable='static_tf',)
    
    dynamic_tf_node = Node(name='dynamic_tf',
                        package='tf_examples',
                        executable='dynamic_tf',)
    
    transform_listener_node = Node(package='tf_examples',
                                executable='tf_listener',
                                output='screen',)
    
    rqt_tf_tree_node = Node(package='rqt_tf_tree',
                            executable='rqt_tf_tree',)
    
    rviz_node = Node(package='rviz2',
                    executable='rviz2',)
    
    l_d = LaunchDescription([static_transform_node, static_transform_node_2, static_transform_node_3, 
                             dynamic_tf_node, transform_listener_node, rqt_tf_tree_node, rviz_node])
    return l_d