from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_odom_transform_node = Node(name="map_odom_transform",
                                    package="tf2_ros",
                                    executable="static_transform_publisher",
                                    arguments=["2", "1", "0", "0", "0", "0", "map", "odom"],)
    
    puzzledrone_node = Node(name="puzzledrone",
                            package="markers",
                            executable="puzzledrone",
                            output="screen")
    
    rviz_node = Node(name="rviz2",
                     package="rviz2",
                     executable="rviz2",
                     output="screen")
    
    rqt_tf_tree_node = Node(name="rqt_tf_tree",
                            package="rqt_tf_tree",
                            executable="rqt_tf_tree",
                            output="screen")
    
    rqt_graph_node = Node(name="rqt_graph",
                            package="rqt_graph",
                            executable="rqt_graph",
                            output="screen")
    
    return LaunchDescription([map_odom_transform_node, puzzledrone_node, rviz_node, rqt_tf_tree_node, rqt_graph_node])