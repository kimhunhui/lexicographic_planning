#ifndef LEX_PLANNER_HPP
#define LEX_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

namespace lex_planner {

class LEXPlanner : public nav2_core::GlobalPlanner, public rclcpp::Node {
public:
    LEXPlanner();
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;
    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) override;

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubTwistCommand_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::Path globalPath_;

    void pathHandler(const nav_msgs::msg::Path::SharedPtr path_msg);
    void twistCommandHandler(const nav_msgs::msg::Path::SharedPtr path_msg);
};

}  // namespace lex_planner

#endif  // LEX_PLANNER_HPP
