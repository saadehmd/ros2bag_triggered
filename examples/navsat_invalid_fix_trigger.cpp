#include <examples/navsat_invalid_fix_trigger.hpp>

using namespace ros2bag_triggered::examples;

bool NavSatInvalidFixTrigger::isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
{
    return msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX || 
           msg->status.service == sensor_msgs::msg::NavSatStatus::SERVICE_UNKNOWN;
}

void NavSatInvalidFixTrigger::fromYaml(const YAML::Node& node) 
{   
    // YAML initialization shouldn't fallback on default constructed values and it should actively throw exceptions 
    // on problems parsing the configuration from yaml. Inclduing missing key-value pairs or wrong access-types.
    try
    {
        persistance_duration_ =  rclcpp::Duration::from_seconds(node["persistance_duration"].as<double>());
        use_msg_stamp_ = node["use_msg_stamp"].as<bool>();
    }
    catch(const YAML::Exception& e)
    {
        std::cerr << "Exception in parsing "<< getName() << " from YAML: " << e.what() << '\n';
        throw e;
    }
    
}