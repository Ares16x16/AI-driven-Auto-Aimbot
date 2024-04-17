from setuptools import find_packages, setup
import os
import glob

package_name = 'ros2_pcnode_test'

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
    maintainer='benny233',
    maintainer_email='benny233@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = ros2_pcnode_test.test_node:main",
            "draw_circle = ros2_pcnode_test.draw_circle:main",
            "cam = ros2_pcnode_test.ros2_pc_node:main",
            "cam_real = ros2_pcnode_test.test_cam:main",
        ],
    },
)