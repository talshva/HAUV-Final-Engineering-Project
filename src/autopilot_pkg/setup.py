from setuptools import setup
import os
from glob import glob

package_name = 'autopilot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tal_Shvartzberg',
    maintainer_email='talshva@post.bgu.ac.il',
    description='package for ROS 2 autopilot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guidance_node = autopilot_pkg.guidance_node:main',
            'imu_node = autopilot_pkg.imu_node:main'
        ],
    },
)
