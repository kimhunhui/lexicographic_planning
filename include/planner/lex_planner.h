#ifndef LEX_PLANNER_HPP
#define LEX_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>

namespace lex_planner {

class LEXPlanner : public nav2_core::GlobalPlanner {
public:
    LEXPlanner();
    explicit LEXPlanner(const rclcpp::Node::SharedPtr& node);
    void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent,
                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr pathMsg);
    void twistCommandCallback(const geometry_msgs::msg::Twist::SharedPtr twistMsg);

    bool createPlan(const geometry_msgs::msg::PoseStamped& start,
                    const geometry_msgs::msg::PoseStamped& goal,
                    std::vector<geometry_msgs::msg::PoseStamped>& plan) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubGoal_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubTwistCommand_;

    std::mutex mutex_;
    nav_msgs::msg::Path globalPath_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
};

}

#endif // LEX_PLANNER_HPP
