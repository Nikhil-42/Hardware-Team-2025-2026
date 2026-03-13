#include "behaviors/map_correction_node.hpp"
#include <cmath>

bool MapCorrectionNode::setRequest(Request::SharedPtr & request)
{
    double x, y, yaw;
    getInput("known_x",   x);
    getInput("known_y",   y);
    getInput("known_yaw", yaw);

    // Build a geometry_msgs::msg::Pose from the known 2D field position
    request->known_map_pose.position.x = x;
    request->known_map_pose.position.y = y;
    request->known_map_pose.position.z = 0.0;

    // Convert yaw to quaternion
    request->known_map_pose.orientation.x = 0.0;
    request->known_map_pose.orientation.y = 0.0;
    request->known_map_pose.orientation.z = std::sin(yaw / 2.0);
    request->known_map_pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO(logger(), "MapCorrectionNode: requesting correction to x=%.3f y=%.3f yaw=%.3f",
                x, y, yaw);
    return true;
}

BT::NodeStatus MapCorrectionNode::onResponseReceived(const Response::SharedPtr & response)
{
    if (response->success)
    {
        RCLCPP_INFO(logger(), "MapCorrectionNode: map->odom correction applied successfully.");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_WARN(logger(), "MapCorrectionNode: correction service returned failure.");
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus MapCorrectionNode::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "%s: onFailure with error: %s (%d)",
                 name().c_str(), BT::toStr(error), static_cast<int>(error));
    return BT::NodeStatus::FAILURE;
}
