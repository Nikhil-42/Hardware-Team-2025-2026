from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	start_light_node = Node(
		package='camera_utils',
		executable='start_light_node',
	)

	return LaunchDescription([
        start_light_node
	])
