cmake_minimum_required(VERSION 3.8)
project(drive_to_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dtp_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(drive_to_pose_server
	src/drive_to_pose_action_server.cpp)

add_executable(drive_to_pose_client
	src/drive_to_pose_action_client.cpp)

target_include_directories(drive_to_pose_server PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
target_include_directories(drive_to_pose_client PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

ament_target_dependencies(drive_to_pose_server dtp_interfaces rclcpp rclcpp_action nav_msgs geometry_msgs turtlesim)
ament_target_dependencies(drive_to_pose_client dtp_interfaces rclcpp rclcpp_action nav_msgs geometry_msgs turtlesim)

install(TARGETS drive_to_pose_server
	DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS drive_to_pose_client
	DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
