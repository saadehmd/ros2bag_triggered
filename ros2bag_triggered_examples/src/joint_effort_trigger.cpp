#include <ros2bag_triggered_examples/joint_effort_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool JointEffortTrigger::isTriggered(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) const
{
    for (const auto& trajectory_point : msg->points)
    {
        for (const auto& effort : trajectory_point.effort)
        {
            if (std::abs(effort) > joint_effort_threshold_)
            {
                return true;
            }
        }
    }
    return false;
}

void JointEffortTrigger::configureConditionalParams(const YAML::Node& node)
{
    joint_effort_threshold_ = node["joint_effort_threshold"].as<double>();
}