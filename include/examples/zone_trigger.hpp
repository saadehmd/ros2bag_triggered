#ifndef ZONE_TRIGGER_HPP
#define ZONE_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ros2bag_triggered::examples
{

struct TriggerZone
{
    double x_min{0};
    double x_max{0};
    double y_min{0};
    double y_max{0};
    double z_min{0};
    double z_max{0};
};

class ZoneTrigger : public TriggerBase<geometry_msgs::msg::PoseStamped>
{
public:
    ZoneTrigger(uint64_t persistance_duration, const TriggerZone& trigger_zone, const rclcpp::Clock::SharedPtr clock) 
    : TriggerBase(persistance_duration, clock),
      trigger_zone_(trigger_zone) {};
    ~ZoneTrigger() override = default;
   
    bool isTriggered(const geometry_msgs::msg::PoseStamped::SharedPtr msg) override;

private:
    TriggerZone trigger_zone_;
};

} // namespace ros2bag_triggered::examples

#endif // ZONE_TRIGGER_HPP
