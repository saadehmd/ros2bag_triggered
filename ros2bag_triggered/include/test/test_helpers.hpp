#ifndef TEST_HELPERS_HPP
#define TEST_HELPERS_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace ros2bag_triggered::tests
{

class EmptyTrigger : public TriggerBase<std_msgs::msg::Bool>
{
public:
    explicit EmptyTrigger(double persistance_duration, const rclcpp::Clock::SharedPtr clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
    : TriggerBase(persistance_duration, clock, logger, use_msg_stamp) {}
    
    EmptyTrigger() = delete;
    ~EmptyTrigger() override = default;

    EmptyTrigger(const EmptyTrigger&) = default;
    EmptyTrigger& operator=(const EmptyTrigger&) = default;
    EmptyTrigger(EmptyTrigger&&) = default;
    EmptyTrigger& operator=(EmptyTrigger&&) = default;
    
    bool is_triggered(const std_msgs::msg::Bool::SharedPtr msg) const override
    {
        return msg->data; 
    }

    std::string get_name() const override
    {
        return "EmptyTrigger";
    }

    void configure_conditional_params(const YAML::Node&) override
    {
        // No conditional params required for this trigger.
    }
        
};

class BatteryHealthTrigger : public TriggerBase<sensor_msgs::msg::BatteryState>
{
public:
    explicit BatteryHealthTrigger(double persistance_duration, const rclcpp::Clock::SharedPtr clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
    : TriggerBase(persistance_duration, clock, logger, use_msg_stamp) {}

    BatteryHealthTrigger() = delete;
    ~BatteryHealthTrigger() override = default;

    
    bool is_triggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) const override
    {
        return msg->power_supply_health != sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }

    std::string get_name() const override
    {
        return "BatteryHealthTrigger";
    }
    
private:
    void configure_conditional_params(const YAML::Node&) override{}
};

}  // namespace ros2bag_triggered::tests

#endif  // TEST_HELPERS_HPP