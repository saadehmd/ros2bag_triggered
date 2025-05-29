#include <ros2bag_triggered_examples/velocity_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool VelocityTrigger::isTriggered(const nav_msgs::msg::Odometry::SharedPtr msg) const
{
    return std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y) > linear_velocity_threshold_ || 
    msg->twist.twist.angular.z > angular_velocity_threshold_;
}

void VelocityTrigger::configureConditionalParams(const YAML::Node& node)
{
    linear_velocity_threshold_ = node["linear_velocity_threshold"].as<double>();
    angular_velocity_threshold_ = node["angular_velocity_threshold"].as<double>();
}