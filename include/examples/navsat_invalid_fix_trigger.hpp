#ifndef NAVSAT_INVALID_FIX_TRIGGER_HPP
#define NAVSAT_INVALID_FIX_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ros2bag_triggered::examples
{

class NavSatInvalidFixTrigger : public TriggerBase<sensor_msgs::msg::NavSatFix>
{
public:
    NavSatInvalidFixTrigger(uint64_t persistance_duration, const rclcpp::Clock::SharedPtr clock, bool use_msg_stamp)
        : TriggerBase(persistance_duration, clock, use_msg_stamp) {}
    
    NavSatInvalidFixTrigger() = delete;
    ~NavSatInvalidFixTrigger() override = default;
    
    bool isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) override;

    static const std::string name;
};
const std::string NavSatInvalidFixTrigger::name = "NavSatInvalidFixTrigger";
} // namespace ros2bag_triggered::examples

#endif // NAVSAT_INVALID_FIX_TRIGGER_HPP