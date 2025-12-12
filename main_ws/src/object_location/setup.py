from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'object_location'

setup(
    name=package_name,
    version='0.0.0',
    packages= find_packages(), # [package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abcd1234',
    maintainer_email='abcd1234@todo.todo',
    description='Object location package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robo_sync_node = object_location.nodes.robo_sync_node:main',
            'detection_node = object_location.nodes.detection_node:main',
            'distance_node = object_location.nodes.distance_node:main',
            'map_node = object_location.nodes.map_node:main',
            'temp_viewer = object_location.temp_viewer:main',
            'approach_controller_node = object_location.nodes.new_approach_controller_node:main',
            'navigator_node = object_location.nodes.navigator_node:main',
            'map_visualizer = object_location.map_visualizer:main'
        ],
    },
)
