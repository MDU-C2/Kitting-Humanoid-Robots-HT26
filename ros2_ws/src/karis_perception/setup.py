from glob import glob

from setuptools import setup

package_name = "karis_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/meshes", glob("meshes/*.stl") + glob("meshes/*.obj") + glob("meshes/*.ply")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Kim Svedberg",
    maintainer_email="kim.svedberg03@gmail.com",
    description="KARIS perception ROS 2 nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pose_estimator = karis_perception.pose_estimator_node:main",
            "camera_player = karis_perception.camera_player_node:main",
            "make_isaac_meshes = karis_perception.make_isaac_meshes:main",
            "bag_reindex = karis_perception.bag_reindex:main",
        ],
    },
)
