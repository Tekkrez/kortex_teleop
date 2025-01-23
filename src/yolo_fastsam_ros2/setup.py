from setuptools import find_packages, setup

package_name = 'yolo_fastsam_ros2'

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
    maintainer='tekkrez',
    maintainer_email='k2joseph@uwo.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["instance_segmenter = yolo_fastsam_ros2.fastsam_node:main","grasp_requester = yolo_fastsam_ros2.request_grasp_node:main"
        ],
    },
)
