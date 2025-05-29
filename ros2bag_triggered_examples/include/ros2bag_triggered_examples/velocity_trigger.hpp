#ifndef VELOCITY_TRIGGER_HPP
#define VELOCITY_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace ros2bag_triggered::examples
{

class VelocityTrigger : public TriggerBase<nav_msgs::msg::Odometry>
{
public:
    VelocityTrigger(double persistance_duration, const rclcpp::Clock::SharedPtr clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
    : TriggerBase(persistance_duration, clock, logger, use_msg_stamp)
    {}

    ~VelocityTrigger() override = default;
    VelocityTrigger() = delete;

    bool isTriggered(const nav_msgs::msg::Odometry::SharedPtr msg) const override ;

    std::string getName() const override
    {
        return "VelocityTrigger";
    }

private:
    
    void configureConditionalParams(const YAML::Node& node) override;

    double linear_velocity_threshold_;
    double angular_velocity_threshold_;

};

} // namespace ros2bag_triggered::examples
#endif  // VELOCITY_TRIGGER_HPP