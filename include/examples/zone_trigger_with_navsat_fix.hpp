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

    ZoneTriggerWithNavSatFix(uint64_t persistance_duration, const rclcpp::Clock::SharedPtr clock, bool use_msg_stamp, const TriggerZone& trigger_zone) 
    : TriggerBase(persistance_duration, clock, use_msg_stamp),
      trigger_zone_(trigger_zone) {}

    ZoneTriggerWithNavSatFix(const YAML::Node& node)
    {
        fromYaml(node);
    }

    ZoneTriggerWithNavSatFix() = default;
    ~ZoneTriggerWithNavSatFix() override = default;
    
    bool isTriggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const override;

    std::string getName() const override
    {
        return "ZoneTriggerWithNavSatFix";
    }
    
protected:
    void fromYaml(const YAML::Node& node) override;
    TriggerZone trigger_zone_;
};
    

}; // namespace ros2bag_triggered::examples
#endif // ZONE_TRIGGER_WITH_NAVSAT_FIX_HPP