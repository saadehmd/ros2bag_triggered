#ifndef BATTERY_HEALTH_TRIGGER_HPP
#define BATTERY_HEALTH_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace ros2bag_triggered::examples
{

class BatteryHealthTrigger : public TriggerBase<sensor_msgs::msg::BatteryState>
{
public:
    explicit BatteryHealthTrigger(double persistance_duration, const rclcpp::Clock::SharedPtr clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
    : TriggerBase(persistance_duration, clock, logger, use_msg_stamp) {}

    BatteryHealthTrigger() = delete;
    ~BatteryHealthTrigger() override = default;

    
    bool is_triggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) const override;

    std::string get_name() const override
    {
        return "BatteryHealthTrigger";
    }
    
private:
    void configure_conditional_params(const YAML::Node& node) override;
};
}  // namespace ros2bag_triggered::examples

#endif  // BATTERY_HEALTH_TRIGGER_HPP