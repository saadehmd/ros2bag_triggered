#ifndef JOINT_EFFORT_TRIGGER_HPP
#define JOINT_EFFORT_TRIGGER_HPP

#include <ros2bag_triggered/trigger_base.hpp> 
#include <trajectory_msgs/msg/joint_trajectory.hpp>


namespace ros2bag_triggered::examples
{
class JointEffortTrigger : public TriggerBase<trajectory_msgs::msg::JointTrajectory>
{
    public:
        JointEffortTrigger(double persistance_duration, const rclcpp::Clock::SharedPtr clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
        : TriggerBase(persistance_duration, clock, logger, use_msg_stamp)
        {}
    
        ~JointEffortTrigger() override = default;
        JointEffortTrigger() = delete;
    
        bool isTriggered(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) const override;
    
        std::string getName() const override
        {
            return "JointEffortTrigger";
        }
    
    private:
        void configureConditionalParams(const YAML::Node& node) override;
        double joint_effort_threshold_;

};

} // namespace ros2bag_triggered::examples 
#endif  // JOINT_EFFORT_TRIGGER_HPP