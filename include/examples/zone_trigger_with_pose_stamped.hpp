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

    ZoneTriggerWithPoseStamped(uint64_t persistance_duration, const TriggerZone& trigger_zone, const rclcpp::Clock::SharedPtr clock) 
    : TriggerBase(persistance_duration, clock),
      trigger_zone_(trigger_zone) {}

    ~ZoneTriggerWithPoseStamped() override = default;

    bool isTriggered(const geometry_msgs::msg::PoseStamped::SharedPtr msg) override;

    static const std::string name;

private:
    TriggerZone trigger_zone_;
};
const std::string ZoneTriggerWithPoseStamped::name = "ZoneTriggerWithPoseStamped";
} // namespace ros2bag_triggered::examples

#endif // ZONE_TRIGGER_WITH_POSE_STAMPED_HPP
