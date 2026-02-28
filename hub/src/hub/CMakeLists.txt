cmake_minimum_required(VERSION 3.8)
project(hub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hub_interfaces REQUIRED)

add_executable(chassis src/chassis.cpp)
ament_target_dependencies(chassis rclcpp std_msgs nav_msgs)
target_include_directories(chassis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(chassis PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(drive_to_pose src/drive_to_pose.cpp)
ament_target_dependencies(drive_to_pose rclcpp rclcpp_action geometry_msgs nav_msgs hub_interfaces)
target_include_directories(drive_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(drive_to_pose PUBLIC c_std_99 cxx_std_17)

# add_executable(fingers src/fingers.cpp)
# ament_target_dependencies(fingers rclcpp std_msgs nav_msgs)
#  target_include_directories(fingers PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(fingers PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS chassis drive_to_pose
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch res
  DESTINATION share/${PROJECT_NAME})

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
