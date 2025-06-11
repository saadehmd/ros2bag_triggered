#ifndef POSE_COVARIANCE_TRIGGER_HPP
#define POSE_COVARIANCE_TRIGGER_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <ros2bag_triggered/trigger_base.hpp>

namespace ros2bag_triggered::examples
{

class PoseCovarianceTrigger : public TriggerBase<nav_msgs::msg::Odometry>
{
public:
  PoseCovarianceTrigger(
    double persistance_duration, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
  : TriggerBase(persistance_duration, clock, logger, use_msg_stamp)
  {
  }

  ~PoseCovarianceTrigger() override = default;
  PoseCovarianceTrigger() = delete;

  bool is_triggered(const nav_msgs::msg::Odometry::SharedPtr msg) const override;

  std::string get_name() const override { return "PoseCovarianceTrigger"; }

private:
  void configure_conditional_params(const YAML::Node & node) override;

  double position_covariance_threshold_;
  double orientation_covariance_threshold_;
};

}  // namespace ros2bag_triggered::examples
#endif  // POSE_COVARIANCE_TRIGGER_HPP
