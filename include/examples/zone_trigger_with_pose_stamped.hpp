#ifndef ZONE_TRIGGER_WITH_POSE_STAMPED_HPP
#define ZONE_TRIGGER_WITH_POSE_STAMPED_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ros2bag_triggered::examples
{

class ZoneTriggerWithPoseStamped : public TriggerBase<geometry_msgs::msg::PoseStamped>
{
public:
    struct TriggerZone
    {
        double x_min{0};
        double x_max{0};
        double y_min{0};
        double y_max{0};
        double z_min{0};
        double z_max{0};
    };

    ZoneTriggerWithPoseStamped(uint64_t persistance_duration, const rclcpp::Clock::SharedPtr clock, bool use_msg_stamp, const TriggerZone& trigger_zone) 
    : TriggerBase(persistance_duration, clock, use_msg_stamp),
      trigger_zone_(trigger_zone) {}

    ZoneTriggerWithPoseStamped(const YAML::Node& node)
    {
        fromYaml(node);
    }

    ZoneTriggerWithPoseStamped() = default;
    ~ZoneTriggerWithPoseStamped() override = default;

    bool isTriggered(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const override;

    std::string getName() const override
    {
        return "ZoneTriggerWithPoseStamped";
    }
    
protected:
    void fromYaml(const YAML::Node& node) override;
    TriggerZone trigger_zone_;
};

    

} // namespace ros2bag_triggered::examples

#endif // ZONE_TRIGGER_WITH_POSE_STAMPED_HPP
