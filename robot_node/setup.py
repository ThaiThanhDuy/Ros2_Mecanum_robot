from setuptools import find_packages, setup
import os
import glob
package_name = 'robot_node' #very important this is correct.

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
      #  (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duy',
    maintainer_email='just.electric.4.fun@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
      entry_points={
        'console_scripts': [
            'odometry_publisher_node = robot_node.odometry_publisher_node:main', 
            'joint_state_publisher = robot_node.joint_state_publisher:main', 
            'cml_vel_node = robot_node.cml_vel_node:main', 
        ],
    },
)
