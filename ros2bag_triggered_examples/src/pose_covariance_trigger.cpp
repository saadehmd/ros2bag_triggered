#include <ros2bag_triggered_examples/pose_covariance_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool PoseCovarianceTrigger::is_triggered(const nav_msgs::msg::Odometry::SharedPtr msg) const
{
    return msg->pose.covariance[0] > position_covariance_threshold_ ||
           msg->pose.covariance[7] > position_covariance_threshold_ ||
           msg->pose.covariance[14] > position_covariance_threshold_ ||
           msg->pose.covariance[21] > orientation_covariance_threshold_ ||
           msg->pose.covariance[28] > orientation_covariance_threshold_ ||
           msg->pose.covariance[35] > orientation_covariance_threshold_;
}
void PoseCovarianceTrigger::configure_conditional_params(const YAML::Node& node)
{
    position_covariance_threshold_ = node["position_covariance_threshold"].as<double>();
    orientation_covariance_threshold_ = node["orientation_covariance_threshold"].as<double>();
}