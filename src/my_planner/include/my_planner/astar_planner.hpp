#pragma once
#include "my_planner/astar_core.hpp"
#include "nav2_core/global_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
namespace my_planner {
class AStarPlanner: public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,std::string,
                 std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped&,
                                const geometry_msgs::msg::PoseStamped&) override;
private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr statistics_;
  std::string name_,frame_;
  Options options_;
};
}
