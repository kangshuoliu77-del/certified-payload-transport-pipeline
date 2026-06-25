from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("swarm_random_payload"))
    case_file = str(package_share / "data" / "demo1_case.json")
    return LaunchDescription(
        [
            Node(
                package="swarm_random_payload",
                executable="random_payload_demo",
                name="simple_payload_demo",
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
                        "case_file": case_file,
                    }
                ],
            )
        ]
    )
