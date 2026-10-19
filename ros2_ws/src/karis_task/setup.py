from setuptools import setup

package_name = "karis_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kim Svedberg",
    maintainer_email="kim.svedberg03@gmail.com",
    description="KARIS motion-side nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pose_listener = karis_task.pose_listener:main",
            "scene_updater = karis_task.scene_updater:main",
            "pick_server = karis_task.pick_server:main",
        ],
    },
)
