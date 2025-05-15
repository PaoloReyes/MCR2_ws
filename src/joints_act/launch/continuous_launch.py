import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_filename = 'continuous_ex.urdf'
    urdf_path = os.path.join(get_package_share_directory('joints_act'), 'urdf', urdf_filename)

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    robot_state_publisher_node = Node(name='robot_state_publisher',
                                    package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    output='screen',
                                    parameters=[{'robot_description': robot_description}],)
    
    joint_state_publisher_node = Node(name='joint_state_publisher',
                                      package='joint_state_publisher_gui',
                                      executable='joint_state_publisher_gui',
                                      output='screen',)

    l_d = LaunchDescription([robot_state_publisher_node, joint_state_publisher_node])

    return l_d