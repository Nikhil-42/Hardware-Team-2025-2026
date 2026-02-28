from launch.substitutions import LaunchConfiguration, Command
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os

def generate_launch_description():
	pkg_share = FindPackageShare(package='hub').find('hub')
	model_path = os.path.join(pkg_share, 'res', 'robot.urdf.xacro')
	ekf_path = os.path.join(pkg_share, 'res', 'ekf.yaml')
	robot_description = ParameterValue(Command(['xacro ', model_path]), value_type=str)

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		parameters=[{'robot_description': robot_description}],
	)
	joint_state_publisher_node = Node(
		package='joint_state_publisher',
		executable='joint_state_publisher',
		name='joint_state_publisher',
		parameters=[{'robot_description': robot_description}],
	)
	robot_localization_node = Node(
		package = 'robot_localization',
		executable='ekf_node',
			name='ekf_filter_node',
			output='screen',
			parameters=[ekf_path]
	)
	chassis_node = Node(
		package = 'hub',
		executable = 'chassis',
		name = 'chassis_node',
		output = 'screen'
	)
	drive_to_pose_server = Node(
		package = 'hub',
		executable = 'drive_to_pose',
		name = 'drive_to_pose_server',
		output = 'screen'
	)
	camera_node = Node(
		package = 'camera_ros',
		executable = 'camera_node',
		name = 'camera',
		output = 'screen',
	)
	start_led_server = Node(
		package = 'camera_utils',
		executable = 'start_light_node',
		name = 'start_light_node',
		output = 'screen',
		parameters=[{
			'image_topic': '/camera/image_raw',
			'threshold': 800.0
		}],
	)
	services = Node(
		package = 'py_hub',
		executable = 'service',
		name = 'services',
		output = 'screen'
	)
	static_tf = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
	)

	enable = Node(
		package='py_hub',
		executable='enable',
		name='enable',
		output='screen',
		arguments=['1']
	)


	return LaunchDescription([
		# set true if running a simulation (uses system clock when false)
		static_tf,
		enable,
		# joint_state_publisher_node,
		robot_localization_node,
		chassis_node,
		drive_to_pose_server,
		camera_node,
		start_led_server,
		services,
		robot_state_publisher_node,
	])
