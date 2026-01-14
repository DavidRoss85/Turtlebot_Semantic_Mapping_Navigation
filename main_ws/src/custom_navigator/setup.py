from setuptools import find_packages, setup

package_name = 'custom_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-ross',
    maintainer_email='ross.d2@northeastern.edu',
    description='Custom navigator package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'approach_controller_node = custom_navigator.nodes.new_approach_controller_node:main',
            'navigator_node = custom_navigator.nodes.navigator_node:main',
            'navigation_server = custom_navigator.nodes.navigation_server:main',
            'server_test = custom_navigator.tools.server_test:main',
        ],
    },
)
