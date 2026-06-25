from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="swarm_random_payload",
                executable="random_payload_demo",
                name="random_payload_demo",
                output="screen",
                parameters=[
                    {
                        "dt": 0.08,
                        "frames_per_tick": 1,
                        "loop_demo": True,
                        "publish_frame": "map",
                        "show_region_labels": True,
                        "show_bridge_labels": True,
                        "show_formation_layer_graph": True,
                        "show_formation_graph_edges": False,
                        "show_symbolic_route": False,
                        "show_current_status_text": False,
                        "publish_centroid_path": False,
                        "map_scale": 0.01,
                    }
                ],
            )
        ]
    )
