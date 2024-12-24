#ifndef EMPTY_TRIGGER_HPP
#define EMPTY_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <std_msgs/msg/empty.hpp>

namespace ros2bag_triggered::examples
{

class EmptyTrigger : public TriggerBase<std_msgs::msg::Empty>
{
public:
    explicit EmptyTrigger(uint64_t persistance_duration, const rclcpp::Clock::SharedPtr clock, bool use_msg_stamp)
    : TriggerBase(persistance_duration, clock, use_msg_stamp) {}

    EmptyTrigger(const YAML::Node& node)
    {
        fromYaml(node);
    }
    
    EmptyTrigger() = default;
    ~EmptyTrigger() override = default;
    
    bool isTriggered(const std_msgs::msg::Empty::SharedPtr msg) const override
    {
        return false; 
    }

    std::string getName() const override
    {
        return name;
    }
    
    static const std::string name;

protected:
    void fromYaml(const YAML::Node& node) override
    {}
};
const std::string EmptyTrigger::name = "EmptyTrigger";
}  // namespace ros2bag_triggered::examples

#endif  // EMPTY_TRIGGER_HPP