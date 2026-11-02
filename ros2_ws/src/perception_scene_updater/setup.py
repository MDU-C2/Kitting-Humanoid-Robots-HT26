from setuptools import setup

package_name = "perception_scene_updater"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Kim Svedberg",
    maintainer_email="kim.svedberg03@gmail.com",
    description="KARIS perception parts -> MoveIt planning scene",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scene_updater = perception_scene_updater.scene_updater_node:main",
        ],
    },
)
