// lex_planner.hpp
#ifndef LEX_PLANNER_HPP
#define LEX_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_core/global_planner.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <mutex>
#include <memory>

namespace lex_planner {

class LEXPlanner : public nav2_core::GlobalPlanner, public rclcpp::Node {
public:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGoal;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subTwistCommand1; // Twist command from move_base
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subTwistCommand2; // Twist command from move_base
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubTwistCommand; // Adjust twist command height to show above the robot

    nav_msgs::msg::Path globalPath;

    std::mutex mtx;

    LEXPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    void initialize(std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void pathHandler(const nav_msgs::msg::Path::SharedPtr pathMsg);
    void twistCommandHandler(const geometry_msgs::msg::Twist::SharedPtr twistMsg);

    bool makePlan(const geometry_msgs::msg::PoseStamped& start, 
                  const geometry_msgs::msg::PoseStamped& goal, 
                  std::vector<geometry_msgs::msg::PoseStamped>& plan) override;
};

} // namespace lex_planner

#endif // LEX_PLANNER_HPP
