from setuptools import setup
import os
from glob import glob

package_name = 'motors_eden'
submodules = "motors_eden/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alessandrini',
    maintainer_email='antoine.alessandrini@centrale.centralelille.fr',
    description='ROS2 Package for dynamixels',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = motors_eden.RosMotorsNode:main ',
        ],
    },
)
