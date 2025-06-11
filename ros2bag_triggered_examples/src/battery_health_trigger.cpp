#include <ros2bag_triggered_examples/battery_health_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool BatteryHealthTrigger::is_triggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) const
{
  return msg->power_supply_health != sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
}

void BatteryHealthTrigger::configure_conditional_params(const YAML::Node &)
{
  // No conditional params required for this trigger.
}
