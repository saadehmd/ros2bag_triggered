#ifndef ZONE_TRIGGER_WITH_NAVSAT_FIX_HPP
#define ZONE_TRIGGER_WITH_NAVSAT_FIX_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ros2bag_triggered::examples
{

class ZoneTriggerWithNavSatFix : public TriggerBase<sensor_msgs::msg::NavSatFix>
{
public:

    struct TriggerZone
    {
        double latitude_min{0};
        double latitude_max{0};
        double longitude_min{0};
        double longitude_max{0};
        double altitude_min{0};
        double altitude_max{0};
    };

    ZoneTriggerWithNavSatFix(double persistance_duration, const rclcpp::Clock::SharedPtr clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp) 
    : TriggerBase(persistance_duration, clock, logger, use_msg_stamp) {}

    ZoneTriggerWithNavSatFix() = delete;
    ~ZoneTriggerWithNavSatFix() override = default;
    
    bool isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const override;

    std::string getName() const override
    {
        return "ZoneTriggerWithNavSatFix";
    }
    
    void fromYaml(const YAML::Node& node) override;

protected:
    TriggerZone trigger_zone_;
};
    

}; // namespace ros2bag_triggered::examples
#endif // ZONE_TRIGGER_WITH_NAVSAT_FIX_HPP