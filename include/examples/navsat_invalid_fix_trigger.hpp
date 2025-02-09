#ifndef NAVSAT_INVALID_FIX_TRIGGER_HPP
#define NAVSAT_INVALID_FIX_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ros2bag_triggered::examples
{

class NavSatInvalidFixTrigger : public TriggerBase<sensor_msgs::msg::NavSatFix>
{
public:
    NavSatInvalidFixTrigger(double persistance_duration, const rclcpp::Clock::SharedPtr clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
        : TriggerBase(persistance_duration, clock, logger, use_msg_stamp) {}
    
    NavSatInvalidFixTrigger() = delete;
    ~NavSatInvalidFixTrigger() override = default;

    NavSatInvalidFixTrigger(const NavSatInvalidFixTrigger&) = delete;
    NavSatInvalidFixTrigger& operator=(const NavSatInvalidFixTrigger&) = delete;
    NavSatInvalidFixTrigger(NavSatInvalidFixTrigger&&) = default;
    NavSatInvalidFixTrigger& operator=(NavSatInvalidFixTrigger&&) = default;
    
    bool isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const override;

    std::string getName() const override
    {
        return "NavSatInvalidFixTrigger";
    }
    
    void fromYaml(const YAML::Node& node) override;
    
};

} // namespace ros2bag_triggered::examples

#endif // NAVSAT_INVALID_FIX_TRIGGER_HPP