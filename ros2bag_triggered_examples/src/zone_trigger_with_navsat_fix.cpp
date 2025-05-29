#include <ros2bag_triggered_examples/zone_trigger_with_navsat_fix.hpp>

using namespace ros2bag_triggered::examples;

bool ZoneTriggerWithNavSatFix::isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
{
    return msg->latitude >= trigger_zone_.latitude_min && msg->latitude <= trigger_zone_.latitude_max &&
           msg->longitude >= trigger_zone_.longitude_min && msg->longitude <= trigger_zone_.longitude_max &&
           msg->altitude >= trigger_zone_.altitude_min && msg->altitude <= trigger_zone_.altitude_max;
}


void ZoneTriggerWithNavSatFix::configureConditionalParams(const YAML::Node& node) 
{   
    trigger_zone_.latitude_min = node["trigger_zone"]["latitude_min"].as<double>();
    trigger_zone_.latitude_max = node["trigger_zone"]["latitude_max"].as<double>();
    trigger_zone_.longitude_min = node["trigger_zone"]["longitude_min"].as<double>();
    trigger_zone_.longitude_max = node["trigger_zone"]["longitude_max"].as<double>();
    trigger_zone_.altitude_min = node["trigger_zone"]["altitude_min"].as<double>();
    trigger_zone_.altitude_max = node["trigger_zone"]["altitude_max"].as<double>();
    
}