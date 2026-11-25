from setuptools import find_packages, setup

package_name = 'multi_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/line_follower_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/line_follower.launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'rclpy', 'cv_bridge'],
    zip_safe=True,
    maintainer='abiram',
    maintainer_email='abiramvijay12@gmail.com',
    description='Multi-robot colour line following controller for Gazebo Fortress.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_follower_node = multi_robot_controller.line_follower_node:main',
            'race_controls = multi_robot_controller.race_controls:main',
        ],
    },
)
