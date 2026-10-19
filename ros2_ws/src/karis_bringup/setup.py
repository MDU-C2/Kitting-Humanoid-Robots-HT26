from glob import glob

from setuptools import setup

package_name = "karis_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kim Svedberg",
    maintainer_email="kim.svedberg03@gmail.com",
    description="KARIS launch files",
    license="MIT",
)
