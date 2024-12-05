#ifndef BATTERY_HEALTH_TRIGGER_HPP
#define BATTERY_HEALTH_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace ros2bag_triggered::examples
{

class BatteryHealthTrigger : public TriggerBase<sensor_msgs::msg::BatteryState>
{
public:
    explicit BatteryHealthTrigger(uint64_t persistance_duration, const rclcpp::Clock::SharedPtr clock, bool use_msg_stamp)
    : TriggerBase(persistance_duration, clock, use_msg_stamp) {}
    
    BatteryHealthTrigger() = delete;
    ~BatteryHealthTrigger() override = default;
    
    bool isTriggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) override;

    static const std::string name;
    
};
const std::string BatteryHealthTrigger::name = "BatteryHealthTrigger";
}  // namespace ros2bag_triggered::examples

#endif  // BATTERY_HEALTH_TRIGGER_HPP