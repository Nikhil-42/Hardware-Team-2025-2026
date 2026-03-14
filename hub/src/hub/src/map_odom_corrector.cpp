#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>


#include "hub_interfaces/srv/set_map_pose.hpp"

/*
 * MapOdomCorrector
 *
 * Exposes a service "set_map_correction" that accepts a known map-frame pose.
 * When called (e.g. from a BT node when the robot reaches a known field
 * position), it recomputes the map -> odom transform to eliminate accumulated
 * odometry drift.
 *
 * Broadcasts the map -> odom transform at 1 Hz continuously.
 *
 * TF tree maintained:
 *   map -> odom  (published here at 1 Hz)
 *   odom -> base_link  (published by your Kalman filter / robot_localization)
 */

class MapOdomCorrector : public rclcpp::Node
{
public:
    MapOdomCorrector() : Node("map_odom_corrector")
    {
        // Declare parameters for known starting position on the field
        declare_parameter("initial_x",   0.13795);
        declare_parameter("initial_y",   0.123825);
        declare_parameter("initial_yaw", M_PI);

        double init_x   = get_parameter("initial_x").as_double();
        double init_y   = get_parameter("initial_y").as_double();
        double init_yaw = get_parameter("initial_yaw").as_double();

        // Initialize TF2
        tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Build initial map -> odom transform from known starting pose
        map_to_odom_.header.frame_id = "map";
        map_to_odom_.child_frame_id  = "odom";
        set_transform_from_2d(map_to_odom_, init_x, init_y, init_yaw);

        RCLCPP_INFO(get_logger(),
            "Initial map->odom set to x=%.3f y=%.3f yaw=%.3f", init_x, init_y, init_yaw);

        // Service server: BT node calls this when robot reaches a known field position
        correction_service_ = create_service<hub_interfaces::srv::SetMapPose>(
            "set_map_correction",
            [this](
                const std::shared_ptr<hub_interfaces::srv::SetMapPose::Request> request,
                std::shared_ptr<hub_interfaces::srv::SetMapPose::Response> response)
            {
                apply_correction(request, response);
            });

        // Broadcast map -> odom at 1 Hz
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { broadcast_transform(); });

        RCLCPP_INFO(get_logger(), "MapOdomCorrector started. Service: /set_map_correction");
    }

private:
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Service<hub_interfaces::srv::SetMapPose>::SharedPtr correction_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped map_to_odom_;

    // Service callback: receives a known map-frame pose and recomputes map -> odom
    //
    // Math:
    //   T_map_base  = known pose of robot in map frame (the landmark)
    //   T_odom_base = current odometry pose (odom -> base_link from TF)
    //   T_map_odom  = T_map_base * inv(T_odom_base)
    void apply_correction(
        const std::shared_ptr<hub_interfaces::srv::SetMapPose::Request> request,
        std::shared_ptr<hub_interfaces::srv::SetMapPose::Response> response)
    {
        // Get current odom -> base_link from TF tree
        geometry_msgs::msg::TransformStamped odom_to_base;
        try
        {
            odom_to_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(get_logger(), "Could not get odom->base_link: %s", ex.what());
            response->success = false;
            return;
        }

        // Build tf2 transform for known map pose from service request
	// T_map_base is the pose of the robot in the map frame
        tf2::Transform T_map_base;
        tf2::fromMsg(request->known_map_pose, T_map_base);

        // Build tf2 transform for current odom pose
        // The pose according to odom (may have drifted)
        tf2::Transform T_odom_base;
        T_odom_base.setOrigin(tf2::Vector3(
            odom_to_base.transform.translation.x,
            odom_to_base.transform.translation.y,
            odom_to_base.transform.translation.z));
        tf2::Quaternion q;
        tf2::fromMsg(odom_to_base.transform.rotation, q);
        T_odom_base.setRotation(q);

        // Compute corrected map -> odom: T_map_odom = T_map_base * inv(T_odom_base)
        tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        // Store the corrected transform
        map_to_odom_.transform.translation.x = T_map_odom.getOrigin().x();
        map_to_odom_.transform.translation.y = T_map_odom.getOrigin().y();
        map_to_odom_.transform.translation.z = T_map_odom.getOrigin().z();
        map_to_odom_.transform.rotation = tf2::toMsg(T_map_odom.getRotation());

        RCLCPP_INFO(get_logger(),
            "Correction applied. New map->odom offset: x=%.4f y=%.4f yaw=%.4f",
            T_map_odom.getOrigin().x(),
            T_map_odom.getOrigin().y(),
            tf2::getYaw(T_map_odom.getRotation()));

        response->success = true;
    }

    void broadcast_transform()
    {
        map_to_odom_.header.stamp = now();
        tf_broadcaster_->sendTransform(map_to_odom_);
    }

    void set_transform_from_2d(
        geometry_msgs::msg::TransformStamped & transform,
        double x, double y, double yaw)
    {
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        transform.transform.rotation = tf2::toMsg(q);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomCorrector>());
    rclcpp::shutdown();
    return 0;
}
