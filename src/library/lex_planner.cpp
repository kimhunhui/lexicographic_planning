#include "lex_planner/lex_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace lex_planner {

LEXPlanner::LEXPlanner() : Node("lex_planner"), tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())) {
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void LEXPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    subPath_ = this->create_subscription<nav_msgs::msg::Path>("planning/planning/execute_path", 5, std::bind(&LEXPlanner::pathHandler, this, std::placeholders::_1));
    pubTwistCommand_ = this->create_publisher<nav_msgs::msg::Path>("/twist_command", 5);
}

void LEXPlanner::cleanup() {
    subPath_.reset();
    pubTwistCommand_.reset();
}

void LEXPlanner::activate() {
    // Activate publishers
    pubTwistCommand_->on_activate();
}

void LEXPlanner::deactivate() {
    // Deactivate publishers
    pubTwistCommand_->on_deactivate();
}

nav_msgs::msg::Path LEXPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) {
    nav_msgs::msg::Path plan;
    if (globalPath_.poses.empty()) {
        RCLCPP_INFO(this->get_logger(), "No valid path found.");
        return plan;
    }

    RCLCPP_INFO(this->get_logger(), "A Valid Path Received!");
    plan = globalPath_;
    plan.poses.back().pose.orientation = goal.pose.orientation;

    return plan;
}

void LEXPlanner::pathHandler(const nav_msgs::msg::Path::SharedPtr path_msg) {
    globalPath_ = *path_msg;
}

void LEXPlanner::twistCommandHandler(const nav_msgs::msg::Path::SharedPtr path_msg) {
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        nav_msgs::msg::Path outTwist = *path_msg;

        for (auto& pose : outTwist.poses) {
	pose.pose.position.z = transform.transform.translation.z + 1.0;
	}
    pubTwistCommand_->publish(outTwist);
	} catch (tf2::TransformException &ex) {
	    RCLCPP_ERROR(this->get_logger(), "Could not transform goal to map frame: %s", ex.what());
	}
}
}
