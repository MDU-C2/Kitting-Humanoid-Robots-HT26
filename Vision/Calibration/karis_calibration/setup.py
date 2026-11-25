
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'karis_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustaubuntu',
    maintainer_email='gustaubuntu@todo.todo',
    description='AprilTag-based camera calibration for Unitree G1 robot',
    license='TODO',
    entry_points={
        'console_scripts': [
            'calibration_capture = karis_calibration.calibration_capture:main',
            'calibration_solver = karis_calibration.calibration_solver:main',
        ],
    },
)
