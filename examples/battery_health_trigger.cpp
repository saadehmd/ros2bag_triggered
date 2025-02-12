#include <examples/battery_health_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool BatteryHealthTrigger::isTriggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) const
{
    return msg->power_supply_health != sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
}

void BatteryHealthTrigger::configureConditionalParams(const YAML::Node& node) 
{  
    // No conditional params required for this trigger.
}
