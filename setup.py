import os
from glob import glob
from setuptools import setup

package_name = 'cafe_butler_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # This fixes warning 1 - marker in package index
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@email.com',
    description='Cafe Butler Robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = cafe_butler_robot.robot_controller:main',
            'order_manager = cafe_butler_robot.order_manager:main',
            'confirmation_node = cafe_butler_robot.confirmation_node:main',
        ],
    },
)
