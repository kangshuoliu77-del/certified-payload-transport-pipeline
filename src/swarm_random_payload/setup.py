from glob import glob

from setuptools import find_packages, setup


package_name = "swarm_random_payload"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
        (f"share/{package_name}/data", glob("data/*.json")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/meshes", glob("meshes/*")),
        (f"share/{package_name}/materials/textures", glob("materials/textures/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shuo",
    maintainer_email="shuo@example.com",
    description="ROS 2 random-seed certified formation-aware payload transport visualization.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "random_payload_demo = swarm_random_payload.random_payload_node:main",
            "gazebo_velocity_tracker = swarm_random_payload.gazebo_velocity_tracker:main",
            "gazebo_payload_tether = swarm_random_payload.gazebo_payload_tether:main",
            "gazebo_live_overlay = swarm_random_payload.gazebo_live_overlay:main",
        ],
    },
)
