#ifndef TEST_HELPERS_HPP
#define TEST_HELPERS_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <std_msgs/msg/bool.hpp>

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
    
    bool isTriggered(const std_msgs::msg::Bool::SharedPtr msg) const override
    {
        return msg->data; 
    }

    std::string getName() const override
    {
        return "EmptyTrigger";
    }

    void configureConditionalParams(const YAML::Node& node) override
    {
        // No conditional params required for this trigger.
    }
        
};
}  // namespace ros2bag_triggered::tests

#endif  // TEST_HELPERS_HPP