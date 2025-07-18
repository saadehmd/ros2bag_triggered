#ifndef ZONE_TRIGGER_WITH_POSE_STAMPED_HPP
#define ZONE_TRIGGER_WITH_POSE_STAMPED_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ros2bag_triggered/trigger_base.hpp>

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

  ZoneTriggerWithPoseStamped(
    double persistance_duration, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
  : TriggerBase(persistance_duration, clock, logger, use_msg_stamp)
  {
  }

  ZoneTriggerWithPoseStamped() = delete;
  ~ZoneTriggerWithPoseStamped() override = default;

  bool is_triggered(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const override;

  std::string get_name() const override { return "ZoneTriggerWithPoseStamped"; }

private:
  void configure_conditional_params(const YAML::Node & node) override;
  TriggerZone
    trigger_zone_;  // Zone to activate the trigger in. An example of conditional parameters.
};

}  // namespace ros2bag_triggered::examples

#endif  // ZONE_TRIGGER_WITH_POSE_STAMPED_HPP
