#include <examples/battery_health_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool BatteryHealthTrigger::isTriggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) const
{
    return msg->power_supply_health != sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
}

void BatteryHealthTrigger::fromYaml(const YAML::Node& node) 
{   
    // YAML initialization shouldn't fallback on default constructed values and it should actively throw exceptions 
    // on problems parsing the configuration from yaml. Inclduing missing key-value pairs or wrong access-types.
    try
    {
        persistance_duration_ =  rclcpp::Duration::from_seconds(node["persistance_duration"].as<uint64_t>());
        use_msg_stamp_ = node["use_msg_stamp"].as<bool>();
    }
    catch(const YAML::Exception& e)
    {
        std::cerr << "Exception in parsing "<< name << " from YAML: " << e.what() << '\n';
    }
    
}
