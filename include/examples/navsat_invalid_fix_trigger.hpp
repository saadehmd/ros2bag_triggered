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

    NavSatInvalidFixTrigger(const YAML::Node& node)
    {
        fromYaml(node);
    }
    
    NavSatInvalidFixTrigger() = default;
    ~NavSatInvalidFixTrigger() override = default;
    
    bool isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const override;

    std::string getName() const override
    {
        return "NavSatInvalidFixTrigger";
    }
    
protected:
    void fromYaml(const YAML::Node& node) override;
};

} // namespace ros2bag_triggered::examples

#endif // NAVSAT_INVALID_FIX_TRIGGER_HPP