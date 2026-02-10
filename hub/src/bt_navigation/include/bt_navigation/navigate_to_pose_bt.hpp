#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/PoseStamped.hpp>

using namespace BT;

class NavigateToPoseBT : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
        NavigateToPoseBT(const std::string& name,
                        const NodeConfig& config,
                        const RosNodeParams& params)
        : RosActionNode<nav2_msgs::action::NavigateToPose(name, config, params) {}

        // takes a pose input as the goal
        static PortsList providedPorts()
        {
                return providedBasicPorts({InputPort<geometry_msgs::msg::PoseStamped>("goal")});
        }

        // called with tree node is ticked and should send req to action server
        bool setGoal(RosActionNode::Goal& goal) override
        {
                // goal.pose is part of nav2_msgs
                getInput("goal", goal.pose);
                return true;
        }

        // callback when reply is recieved
        // return success or failure depending on result
        NoteStatus onResultRecieved(const WrappedResult& wr) override
        {
                if(wr.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                        return NodeStatus::SUCCESS;
                }
                else
                {
                        return NodeStatus::FAILURE;
                }
        }

        // returns feedback information while navigating to pose
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
                RCLCPP_INFO(logger(), "Navigating to pose...");
                return NodeStatus::RUNNING;
        }
};
