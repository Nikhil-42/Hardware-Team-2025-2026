cmake_minimum_required(VERSION 3.16)
project(behaviors)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hub_interfaces REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_ros2::behaviortree_ros2
    ${rclcpp_TARGETS}
    ${btcpp_ros2_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${hub_interfaces_TARGETS} )

######################################################
# Static library for custom TreeNodes
add_library(behaviors_nodes STATIC
  src/drive_to_pose_node.cpp
  src/finger_node.cpp
)
target_include_directories(behaviors_nodes PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(behaviors_nodes ${THIS_PACKAGE_DEPS})

######################################################
# Simple example showing how to use and customize the BtExecutionServer
add_executable(bt_executor src/bt_executor.cpp)
target_include_directories(bt_executor PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(bt_executor behaviors_nodes)
target_link_libraries(bt_executor ${THIS_PACKAGE_DEPS})

######################################################
# Build the (Plugin)

add_library(bt_nodes SHARED src/plugin.cpp)
target_include_directories(bt_nodes PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_definitions(bt_nodes PRIVATE  BT_PLUGIN_EXPORT )
target_link_libraries(bt_executor behaviors_nodes)
target_link_libraries(bt_nodes ${THIS_PACKAGE_DEPS})

######################################################
# Build subscriber_test
add_executable(subscriber_test src/subscriber_test.cpp)
target_link_libraries(subscriber_test ${THIS_PACKAGE_DEPS})

######################################################
# INSTALL

install(TARGETS
  bt_nodes 
  subscriber_test
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
  )

######################################################
# INSTALL plugins for other packages to load

install(TARGETS
  bt_nodes
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

######################################################
# INSTALL Behavior.xml's, ROS config and launch files

install(DIRECTORY
    behavior_trees
    config
    launch
    DESTINATION share/${PROJECT_NAME}/
    )


ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()
