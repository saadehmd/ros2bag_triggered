#include <ros2bag_triggered_examples/zone_trigger_with_pose_stamped.hpp>

using namespace ros2bag_triggered::examples;

bool ZoneTriggerWithPoseStamped::is_triggered(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{
  return msg->pose.position.x >= trigger_zone_.x_min &&
         msg->pose.position.x <= trigger_zone_.x_max &&
         msg->pose.position.y >= trigger_zone_.y_min &&
         msg->pose.position.y <= trigger_zone_.y_max &&
         msg->pose.position.z >= trigger_zone_.z_min && msg->pose.position.z <= trigger_zone_.z_max;
}

void ZoneTriggerWithPoseStamped::configure_conditional_params(const YAML::Node & node)
{
  trigger_zone_.x_min = node["trigger_zone"]["min_x"].as<double>();
  trigger_zone_.x_max = node["trigger_zone"]["max_x"].as<double>();
  trigger_zone_.y_min = node["trigger_zone"]["min_y"].as<double>();
  trigger_zone_.y_max = node["trigger_zone"]["max_y"].as<double>();
  trigger_zone_.z_min = node["trigger_zone"]["min_z"].as<double>();
  trigger_zone_.z_max = node["trigger_zone"]["max_z"].as<double>();
}
