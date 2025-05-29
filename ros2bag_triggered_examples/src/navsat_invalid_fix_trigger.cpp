#include <ros2bag_triggered_examples/navsat_invalid_fix_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool NavSatInvalidFixTrigger::isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
{
    return msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX || 
           msg->status.service == sensor_msgs::msg::NavSatStatus::SERVICE_UNKNOWN;
}

void NavSatInvalidFixTrigger::configureConditionalParams(const YAML::Node&) 
{  
    // No conditional params required for this trigger.
}
