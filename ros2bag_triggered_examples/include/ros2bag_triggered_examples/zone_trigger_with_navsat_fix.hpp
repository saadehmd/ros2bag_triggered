#ifndef ZONE_TRIGGER_WITH_NAVSAT_FIX_HPP
#define ZONE_TRIGGER_WITH_NAVSAT_FIX_HPP

#include <ros2bag_triggered/trigger_base.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace ros2bag_triggered::examples
{

class ZoneTriggerWithNavSatFix : public TriggerBase<sensor_msgs::msg::NavSatFix>
{
public:
  struct TriggerZone
  {
    double latitude_min{0};
    double latitude_max{0};
    double longitude_min{0};
    double longitude_max{0};
    double altitude_min{0};
    double altitude_max{0};
  };

  ZoneTriggerWithNavSatFix(
    double persistance_duration, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
  : TriggerBase(persistance_duration, clock, logger, use_msg_stamp)
  {
  }

  ZoneTriggerWithNavSatFix() = delete;
  ~ZoneTriggerWithNavSatFix() override = default;

  bool is_triggered(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const override;

  std::string get_name() const override { return "ZoneTriggerWithNavSatFix"; }

private:
  void configure_conditional_params(const YAML::Node & node) override;
  TriggerZone
    trigger_zone_;  // Zone to activate the trigger in. An example of conditional parameters.
};

}  // namespace ros2bag_triggered::examples
#endif  // ZONE_TRIGGER_WITH_NAVSAT_FIX_HPP
