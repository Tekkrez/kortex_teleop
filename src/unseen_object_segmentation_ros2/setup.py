from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'unseen_object_segmentation_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'experiments'), glob(os.path.join('experiments','cfgs', '*.yml'))),
        (os.path.join('share', package_name, 'data'), glob(os.path.join('data','checkpoints', '*.pth'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tekkrez',
    maintainer_email='k2joseph@uwo.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["scene_segmenter = unseen_object_segmentation_ros2.scene_segmentation_node:main"
        ],
    },
)
