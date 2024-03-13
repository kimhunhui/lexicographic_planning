#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>

using std::placeholders::_1;

namespace lex_planner {

class LEXPlanner : public nav2_core::GlobalPlanner {
public:
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGoal;

    // For visualizing twist commands
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subTwistCommand1;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subTwistCommand2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubTwistCommand;

    nav_msgs::msg::Path globalPath;

    std::mutex mtx;

    LEXPlanner()
    : LEXPlanner("", nullptr) {}

    LEXPlanner(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf)
    : tf_buffer_(tf) {
        (void)name; // Unused parameter for now
    }

    void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent,
                   std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {
        node_ = parent;
        tf_buffer_ = tf;
        // Initialize subscriptions, publishers, etc.
    }

    void pathHandler(const nav_msgs::msg::Path::SharedPtr pathMsg) {
        // Handle path
    }

    // Visualize twist command
    void twistCommandHandler(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Handle twist command
    }

    bool makePlan(const geometry_msgs::msg::PoseStamped& start,
                  const geometry_msgs::msg::PoseStamped& goal,
                  std::vector<geometry_msgs::msg::PoseStamped>& plan) override {
        // Implement planning logic
        return true;
    }
};

}; // namespace lex_planner

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lex_planner::LEXPlanner)
