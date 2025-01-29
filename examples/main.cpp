#include <iostream>
#include <examples/battery_health_trigger.hpp>
#include <examples/navsat_invalid_fix_trigger.hpp>
#include <examples/zone_trigger_with_pose_stamped.hpp>
#include <examples/zone_trigger_with_navsat_fix.hpp>
#include <ros2bag_triggered/triggered_recorder_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace ros2bag_triggered;

// std::monostate should be used as the first type, to represent the absence of a value in a variant.

using MyVariant = std::variant<std::monostate, examples::ZoneTriggerWithPoseStamped, examples::ZoneTriggerWithNavSatFix, examples::NavSatInvalidFixTrigger, examples::BatteryHealthTrigger>;

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2bag_triggered::TriggeredRecorderNode<MyVariant>>("triggered_recorder_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}