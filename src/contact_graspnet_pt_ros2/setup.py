from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'contact_graspnet_pt_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'checkpoints'), glob(os.path.join('checkpoints','contact_graspnet','*'))),
        (os.path.join('share', package_name, 'gripper_models'), glob(os.path.join('gripper_models','panda_gripper','*.stl'))),
        (os.path.join('share', package_name, 'gripper_control_points'), glob(os.path.join('gripper_control_points','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tekkrez',
    maintainer_email='k2joseph@uwo.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["grasp_generator = contact_graspnet_pt_ros2.cgn_node:main",
                            "test_cgn = contact_graspnet_pt_ros2.test_cgn:main"
        ],
    },
)
