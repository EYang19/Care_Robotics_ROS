from setuptools import find_packages, setup

package_name = 'ml_ros_websocket_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/bridge_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'fastapi>=0.104.0',
        'uvicorn>=0.24.0',
        'websockets>=12.0',
        'pydantic>=2.0.0',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Bridge between CareRobotics task allocation and ROS2 Nav2 navigation with inventory/consumption sensor integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Executable named 'ml_ros_websocket_bridge' (matches package.xml),
            # but imports from the actual source dir which is 'robot_ros_bridge/'.
            'ml_ros_websocket_bridge = robot_ros_bridge.main:main',
        ],
    },
)
