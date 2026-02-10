from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

get_package_share_directory('hub')

def generate_launch_description():
	pkg_share = get_package_share_directory('hub')

	robot_localization_node = Node(
		package = 'robot_localization',
		executable='ekf_node',
    		name='ekf_filter_node',
    		output='screen',
    		parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
	)

	chassis_node = Node(
		package = 'hub',
		executable = 'chassis',
		name = 'chassis_node',
		output = 'screen'
	)

	return LaunchDescription([
		# set true if running a simulation (uses system clock when false)
		launch.actions.DeclareLaunchArgument(
			name = 'use_sim_time',
			default_value = 'False',
			description = 'flag to enable use_sim_time'
		),
		robot_localization_node,
		chassis_node
	])
