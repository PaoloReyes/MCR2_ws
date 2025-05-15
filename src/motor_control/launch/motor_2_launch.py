from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node_1 = Node(name="motor_sys",
                        package='motor_control',
                        executable='dc_motor',
                        emulate_tty=True,
                        output='screen',
                        namespace="group1",
                        )
    
    sp_node_1 = Node(name="sp_gen",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       namespace="group1",
                       )
    
    motor_node_2 = Node(name="motor_sys",
                        package='motor_control',
                        executable='dc_motor',
                        emulate_tty=True,
                        output='screen',
                        namespace="group2",
                        )
    
    sp_node_2 = Node(name="sp_gen",
                        package='motor_control',
                        executable='set_point',
                        emulate_tty=True,
                        output='screen',
                        namespace="group2",
                        )
    
    rqt_plot_node = Node(name="rqt_plot",
                        package='rqt_plot',
                        executable='rqt_plot',
                        emulate_tty=True,
                        output='screen',
                        )
    
    rqt_graph_node = Node(name="rqt_graph",
                        package='rqt_graph',
                        executable='rqt_graph',
                        emulate_tty=True,
                        output='screen',
                        )
    
    l_d = LaunchDescription([motor_node_1, sp_node_1,
                             motor_node_2, sp_node_2,
                             rqt_plot_node, rqt_graph_node])

    return l_d