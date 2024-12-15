#include <examples/zone_trigger_with_navsat_fix.hpp>

using namespace ros2bag_triggered::examples;

bool ZoneTriggerWithNavSatFix::isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
{
    return msg->latitude >= trigger_zone_.latitude_min && msg->latitude <= trigger_zone_.latitude_max &&
           msg->longitude >= trigger_zone_.longitude_min && msg->longitude <= trigger_zone_.longitude_max &&
           msg->altitude >= trigger_zone_.altitude_min && msg->altitude <= trigger_zone_.altitude_max;
}