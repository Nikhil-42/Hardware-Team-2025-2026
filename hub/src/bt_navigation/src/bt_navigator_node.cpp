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

			geometry_msgs::msg::PoseStamped goal;
			goal.header.frame_id = "map";
			goal.header.stamp = this->now();
			goal.pose.position.x = 0.25;
			goal.pose.position.y = 0.25;
			goal.pose.orientation.w = 1.0;

			factory.registerBuilder<NavigateToPoseBT>(
				"NavigateToPose",
  				[](const std::string& name, const BT::NodeConfiguration& config)
  				{
    					return std::make_unique<NavigateToPoseBT>(
        				name,
        				"navigate_to_pose",
        				config);
				});

			//blackboard->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
		}

		void initialize()
		{
			blackboard->set("node", this->shared_from_this());
			blackboard->set("bt_loop_duration", std::chrono::milliseconds(50));
			blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(5000));

			std::string pkg_path = ament_index_cpp::get_package_share_directory("bt_navigation");
			tree = factory.createTreeFromFile(pkg_path + "/behavior_trees/nav_tree_test.xml", blackboard);

			timer = this->create_wall_timer(std::chrono::milliseconds(50),
				std::bind(&BTNavigatorNode::tickTree, this));
		}
	private:
		void tickTree()
		{
			tree.tickWhileRunning(std::chrono::milliseconds(10));
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
	node->initialize();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
