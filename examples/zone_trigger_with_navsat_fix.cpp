#include <examples/zone_trigger_with_navsat_fix.hpp>

using namespace ros2bag_triggered::examples;

bool ZoneTriggerWithNavSatFix::isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
{
    return msg->latitude >= trigger_zone_.latitude_min && msg->latitude <= trigger_zone_.latitude_max &&
           msg->longitude >= trigger_zone_.longitude_min && msg->longitude <= trigger_zone_.longitude_max &&
           msg->altitude >= trigger_zone_.altitude_min && msg->altitude <= trigger_zone_.altitude_max;
}


void ZoneTriggerWithNavSatFix::fromYaml(const YAML::Node& node) 
{   
    // YAML initialization shouldn't fallback on default constructed values and it should actively throw exceptions 
    // on problems parsing the configuration from yaml. Inclduing missing key-value pairs or wrong access-types.
    try
    {
        enabled_ = node["enabled"].as<bool>();
        persistance_duration_ =  rclcpp::Duration::from_seconds(node["persistance_duration"].as<double>());
        use_msg_stamp_ = node["use_msg_stamp"].as<bool>();
        trigger_zone_.latitude_min = node["trigger_zone"]["latitude_min"].as<double>();
        trigger_zone_.latitude_max = node["trigger_zone"]["latitude_max"].as<double>();
        trigger_zone_.longitude_min = node["trigger_zone"]["longitude_min"].as<double>();
        trigger_zone_.longitude_max = node["trigger_zone"]["longitude_max"].as<double>();
        trigger_zone_.altitude_min = node["trigger_zone"]["altitude_min"].as<double>();
        trigger_zone_.altitude_max = node["trigger_zone"]["altitude_max"].as<double>();
    }
    catch(const YAML::Exception& e)
    {
        std::cerr << "Exception in parsing "<< getName() << " from YAML: " << e.what() << '\n';
        throw e;
    }
    
}