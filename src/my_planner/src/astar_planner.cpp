#include "my_planner/astar_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace my_planner {
void AStarPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
 std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_=parent;name_=std::move(name);costmap_ros_=std::move(costmap_ros);
  auto node=node_.lock();if(!node||!costmap_ros_)throw std::runtime_error("AStarPlanner missing parent/costmap");
  frame_=costmap_ros_->getGlobalFrameID();
  rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
  descriptor.description="Applied at configure; restart/reconfigure with new YAML to change.";
  auto parameter=[&](const std::string&key,const rclcpp::ParameterValue&value){
    const auto full=name_+"."+key;
    if(!node->has_parameter(full))node->declare_parameter(full,value,descriptor);
    return node->get_parameter(full);
  };
  options_.eight_connected=parameter("use_8_connected",rclcpp::ParameterValue(true)).as_bool();
  options_.allow_unknown=parameter("allow_unknown",rclcpp::ParameterValue(false)).as_bool();
  const auto unknown=parameter("unknown_cost",rclcpp::ParameterValue(128)).as_int();
  if(unknown<0||unknown>252)throw std::runtime_error("AStarPlanner unknown_cost must be 0..252");
  options_.unknown_cost=static_cast<int>(unknown);
  options_.cost_weight=parameter("cost_weight",rclcpp::ParameterValue(2.0)).as_double();
  options_.min_clearance=parameter("min_obstacle_clearance",rclcpp::ParameterValue(0.0)).as_double();
  options_.max_time=parameter("max_search_time",rclcpp::ParameterValue(1.0)).as_double();
  if(!std::isfinite(options_.cost_weight)||options_.cost_weight<0 ||
     !std::isfinite(options_.min_clearance)||options_.min_clearance<0 ||
     !std::isfinite(options_.max_time)||options_.max_time<=0 ||
     options_.unknown_cost<0||options_.unknown_cost>252)
    throw std::runtime_error("AStarPlanner invalid parameters");
  statistics_=node->create_publisher<std_msgs::msg::String>("~/"+name_+"/statistics",10);
  RCLCPP_INFO(node->get_logger(),"Configured self-developed AStar %s, connectivity=%d, weight=%.3f, extra_clearance=%.3f",
    name_.c_str(),options_.eight_connected?8:4,options_.cost_weight,options_.min_clearance);
}
void AStarPlanner::activate(){if(statistics_)statistics_->on_activate();}
void AStarPlanner::deactivate(){if(statistics_)statistics_->on_deactivate();}
void AStarPlanner::cleanup(){statistics_.reset();costmap_ros_.reset();}

nav_msgs::msg::Path AStarPlanner::createPlan(const geometry_msgs::msg::PoseStamped&start,
 const geometry_msgs::msg::PoseStamped&goal) {
  auto node=node_.lock();if(!node||!costmap_ros_)throw nav2_core::PlannerException("AStar not configured");
  if(start.header.frame_id!=frame_||goal.header.frame_id!=frame_)
    throw nav2_core::PlannerException("AStar requires start and goal in costmap frame");
  if(!std::isfinite(start.pose.position.x)||!std::isfinite(start.pose.position.y)||
     !std::isfinite(goal.pose.position.x)||!std::isfinite(goal.pose.position.y))
    throw nav2_core::PlannerException("AStar non-finite endpoint");
  const auto began=std::chrono::steady_clock::now();
  Grid grid;double ox,oy;unsigned int sx,sy,gx,gy;
  auto *map=costmap_ros_->getCostmap();
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    if(!map->worldToMap(start.pose.position.x,start.pose.position.y,sx,sy)||
       !map->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy))
      throw nav2_core::PlannerException("AStar endpoint outside costmap");
    const auto width=map->getSizeInCellsX(),height=map->getSizeInCellsY();
    if(static_cast<uint64_t>(width)*height>static_cast<uint64_t>(std::numeric_limits<int>::max()))
      throw nav2_core::PlannerException("AStar costmap too large");
    grid.width=static_cast<int>(width);grid.height=static_cast<int>(height);
    grid.resolution=map->getResolution();ox=map->getOriginX();oy=map->getOriginY();
    grid.costs.assign(map->getCharMap(),map->getCharMap()+static_cast<size_t>(width)*height);
  }
  auto options=options_;
  options.robot_radius=costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  const auto result=search(grid,static_cast<int>(sy)*grid.width+sx,
    static_cast<int>(gy)*grid.width+gx,options,[](){return !rclcpp::ok();});
  auto publish=[&](bool success,const std::string&reason) {
    if(!statistics_||!statistics_->is_activated())return;
    const double total=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    std::ostringstream s;s.precision(12);
    s<<"{\"success\":"<<(success?"true":"false")<<",\"reason\":\""<<reason
     <<"\",\"expanded\":"<<result.expanded<<",\"search_ms\":"<<result.elapsed_ms
     <<",\"total_ms\":"<<total<<",\"weighted_cost\":"<<(std::isfinite(result.cost)?result.cost:-1.)<<"}";
    std_msgs::msg::String message;message.data=s.str();statistics_->publish(message);
  };
  if(result.status!=Status::Success){publish(false,result.reason);throw nav2_core::PlannerException("AStar: "+result.reason);}
  std::vector<std::pair<double,double>> points;
  points.emplace_back(start.pose.position.x,start.pose.position.y);
  for(int cell:result.cells)
    points.emplace_back(ox+(cell%grid.width+.5)*grid.resolution,oy+(cell/grid.width+.5)*grid.resolution);
  points.emplace_back(goal.pose.position.x,goal.pose.position.y);
  std::vector<std::pair<double,double>> unique;
  for(const auto&p:points)
    if(unique.empty()||std::hypot(p.first-unique.back().first,p.second-unique.back().second)>1e-10)unique.push_back(p);
  // The same-cell case needs no detour through the center of that cell.
  if(result.cells.size()==1){
    unique.clear();unique.emplace_back(start.pose.position.x,start.pose.position.y);
    if(std::hypot(goal.pose.position.x-start.pose.position.x,goal.pose.position.y-start.pose.position.y)>1e-10)
      unique.emplace_back(goal.pose.position.x,goal.pose.position.y);
  }
  for(size_t i=0;i<unique.size();++i){
    const auto &a=unique[i],&b=unique[i?i-1:i];
    if(!segment_safe(grid,result.traversable,(a.first-ox)/grid.resolution,(a.second-oy)/grid.resolution,
       (b.first-ox)/grid.resolution,(b.second-oy)/grid.resolution)){
      publish(false,"unsafe endpoint connector");throw nav2_core::PlannerException("AStar unsafe connector");
    }
  }
  auto q=goal.pose.orientation;
  const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if(!std::isfinite(norm)||norm<1e-9)throw nav2_core::PlannerException("AStar invalid goal quaternion");
  q.x/=norm;q.y/=norm;q.z/=norm;q.w/=norm;
  nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=node->now();
  for(size_t i=0;i<unique.size();++i) {
    geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
    pose.pose.position.x=unique[i].first;pose.pose.position.y=unique[i].second;
    if(i+1<unique.size()){
      const double yaw=std::atan2(unique[i+1].second-unique[i].second,unique[i+1].first-unique[i].first);
      pose.pose.orientation.z=std::sin(yaw/2);pose.pose.orientation.w=std::cos(yaw/2);
    }else pose.pose.orientation=q;
    path.poses.push_back(pose);
  }
  publish(true,"success");return path;
}
}
PLUGINLIB_EXPORT_CLASS(my_planner::AStarPlanner,nav2_core::GlobalPlanner)
