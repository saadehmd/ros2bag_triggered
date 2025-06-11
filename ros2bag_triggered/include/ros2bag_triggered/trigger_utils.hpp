#ifndef TRIGGER_UTILS_HPP
#define TRIGGER_UTILS_HPP

namespace ros2bag_triggered
{

struct TriggerPulse
{
  TriggerPulse(uint64_t start_time, uint64_t end_time)
  : start_time_(start_time), end_time_(end_time)
  {
  }
  uint64_t start_time_;
  uint64_t end_time_;
};

using TriggerPulseMap = std::unordered_map<std::string, std::vector<TriggerPulse>>;

}  // namespace ros2bag_triggered

#endif  // TRIGGER_UTILS_HPP