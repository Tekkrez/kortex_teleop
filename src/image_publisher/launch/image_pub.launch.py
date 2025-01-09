from launch import LaunchDescription
from launch_ros.actions import Node

#ros2 run image_transport republish raw compressed --ros-args --remap in:=image_raw --remap out:=camera/image
def generate_launch_description():
    imagePublisher = Node(
                package="image_publisher",
                executable="image_publisher"
            )
    compressImage = Node(
                package="image_transport",
                executable="republish",
                arguments=['raw','compressed'],
                remappings=[
                    ('in', 'image_raw')
                    ],
                parameters=[{"out.compressed.jpeg_quality":95,"qos_overrides./parameter_events.publisher.reliability":"best_effort","qos_overrides./parameter_events.publisher.depth": 1}]
            )
    
    return LaunchDescription([
        imagePublisher,
        compressImage
        ]
    )