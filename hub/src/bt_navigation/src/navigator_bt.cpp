#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "bt_navigation/navigate_to_pose_bt.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

using namespace BT;

class BTNavigatorNode : public rclcpp::Node
{
	public:
		BTNavigatorNode() : Node("bt_navigator_node")
		{
			RCLCPP_INFO(this->get_logger(), "BT navigator started");


			blackboard = Blackboard::create();
			blackboard->set("node", this); //allows action node to access this node

			geometry_msgs::msg::PoseStamped goal;
			goal.header.frame_id = "map";
			goal.header.stamp = this->now();
			goal.pose.position.x = 0.25;
			goal.pose.position.y = 0.25;
			goal.pose.orientation.w = 1.0;

			blackboard->set("nav_goal", goal);

			factory.registerNodeType<NavigateToPoseBT>("NavigateToPose");
			std::string pkg_path = ament_index_cpp::get_package_share_directory("bt_navigation");
			tree = factory.createTreeFromFile(pkg_path + "/behavior_trees/nav_tree_test.xml", blackboard);

			timer = this->create_wall_timer(std::chrono::milliseconds(50),
				std::bind(&BTNavigatorNode::tickTree, this));
		}
	private:
		void tickTree()
		{
			tree.tickRoot();
		}

		Tree tree;
		Blackboard::Ptr blackboard;
		BehaviorTreeFactory factory;
		rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<BTNavigatorNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
