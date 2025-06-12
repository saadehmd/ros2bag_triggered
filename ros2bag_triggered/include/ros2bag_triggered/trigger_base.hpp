#ifndef TRIGGER_BASE_HPP
#define TRIGGER_BASE_HPP

#include <yaml-cpp/yaml.h>

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2bag_triggered/trigger_utils.hpp"
#include "ros2bag_triggered/type_traits.hpp"

namespace ros2bag_triggered
{

template <typename T>
class TriggerBase
{
  static_assert(
    type_traits::IsRosIdlType<T>::value,
    "TriggerBase can only be instantiated with ROS2-IDL message types");

public:
  using MsgType = T;
  explicit TriggerBase(
    double persistance_duration, const rclcpp::Clock::SharedPtr & clock,
    const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
  : persistance_duration_(rclcpp::Duration::from_seconds(persistance_duration)),
    clock_(clock),
    logger_(logger),
    use_msg_stamp_(use_msg_stamp)
  {
    if (!(use_msg_stamp_ && type_traits::HasHeader<T>::value) && !clock_)
    {
      throw std::runtime_error("Missing time-source for the trigger.");
    }
  }

  /**
   * @brief Initialize/Re-initialize the trigger from a YAML node.
   *
   * @param node The YAML node containing the trigger configuration.
   */
  void from_yaml(const YAML::Node & node)
  {
    // YAML initialization shouldn't fallback on default constructed values and it should actively throw exceptions
    // on problems parsing the configuration from yaml. Including missing key-value pairs or wrong access-types.
    try
    {
      persistance_duration_ =
        node["persistance_duration"]
          ? rclcpp::Duration::from_seconds(node["persistance_duration"].as<double>())
          : rclcpp::Duration::from_seconds(0);
      use_msg_stamp_ = node["persistance_duration"] ? node["use_msg_stamp"].as<bool>() : false;
      //The base class is still responsible for calling the derived class's configuration of conditional parameters.
      //This is so that the derived class' implementation of yaml-configuration is minimal i.e.; The initialization
      //of required parameters and exception-handling is all done in the base class.
      configure_conditional_params(node);
    } catch (const YAML::BadConversion & e)
    {
      RCLCPP_ERROR(
        *logger_, "Bad-conversion while configuring %s: %s", get_name().c_str(), e.what());
      throw;
    } catch (const YAML::InvalidNode & e)
    {
      RCLCPP_ERROR(*logger_, "Invalid-node while configuring %s: %s", get_name().c_str(), e.what());
      throw;
    }
  }

  TriggerBase() = delete;
  virtual ~TriggerBase() = default;

  /**
   * @brief Check if the trigger is activated by the given message. 
   * This should be implemented by derived classes.
   *
   * @param msg The message to check.
   * @return true if the trigger is activated, false otherwise.
   */
  virtual bool is_triggered(const typename T::SharedPtr msg) const = 0;

  /**
   * @brief Get the name of the trigger. This should be implemented by derived classes.
   */
  virtual std::string get_name() const = 0;

  /**
   * @brief Get the underlying message type of the trigger as a string.
   */
  std::string get_msg_type() const { return rosidl_generator_traits::name<T>(); }

  /**
   * @brief Get trigger's configuration as a human-readable string.
   */
  virtual std::string get_trigger_info() const
  {
    return "\n\tTrigger Type: " + get_name() + "\n\tMsg Type: " + get_msg_type() +
           "\n\tPersistance Duration: " + std::to_string(persistance_duration_.seconds()) +
           " seconds" + "\n\tUsing Message Stamps: " + (use_msg_stamp_ ? "true" : "false") +
           "\n\tEnabled: " + (enabled_ ? "true" : "false") +
           "\n\t==================================================================================="
           "====\n";
  }
  /**
   * @brief Get the trigger's stats i.e.; trigger pulses and duration as human-readable string.
   * @return A string containing the trigger's stats.
   */
  std::string get_trigger_stats() const
  {
    std::string stats = "Trigger Stats for " + get_name() + ":\n";
    for (const auto & pulse : trigger_pulses_)
    {
      auto seconds = std::to_string(
        rclcpp::Duration(std::chrono::nanoseconds(pulse.end_time_ - pulse.start_time_)).seconds());
      stats += "\n\tTriggered from: " + std::to_string(pulse.start_time_ / 1000000000) + "." +
               std::to_string(pulse.start_time_ % 1000000000) + " to " +
               std::to_string(pulse.end_time_ / 1000000000) + "." +
               std::to_string(pulse.end_time_ % 1000000000) + " [" + seconds + " sec.s]\n";
    }
    stats +=
      "\n\t======================================================================================="
      "\n";
    return stats;
  }

  /**
    * @brief Get the trigger's stats i.e.; trigger pulses and duration as json string.
    * This is used for debugging and logging purposes.
    */
  std::string jsonify() const
  {
    std::string json = "{\n";
    json += "\t\"msg_type\": \"" + get_msg_type() + "\",\n";
    json +=
      "\t\"persistance_duration\": " + std::to_string(persistance_duration_.seconds()) + ",\n";
    json += "\t\"use_msg_stamp\": " + std::to_string(use_msg_stamp_) + ",\n";
    json += "\t\"trigger_pulses\": [\n";

    for (const auto & pulse : trigger_pulses_)
    {
      json += "\t\t{\"start_time\": " + std::to_string(pulse.start_time_) +
              ", \"end_time\": " + std::to_string(pulse.end_time_) + "},\n";
    }
    json += "\t]\n}";
    return json;
  }

  std::vector<TriggerPulse> get_trigger_pulses() const { return trigger_pulses_; }

  void reset()
  {
    first_stamp_ = 0;
    last_stamp_ = 0;
    trigger_pulses_.clear();
  }

  /** @brief Set the clock used by the triggers. */
  void set_clock(const rclcpp::Clock::SharedPtr & clock) { clock_ = clock; }

  /** @brief Set the rclcp::logger used by the triggers. */
  void set_logger(const std::shared_ptr<rclcpp::Logger> & logger) { logger_ = logger; }

  /** @brief Check if the trigger is configured to use message stamps. */
  bool is_using_msg_stamps() const { return use_msg_stamp_; }

  bool is_enabled() const { return enabled_; }

  void set_enabled(bool enabled) { enabled_ = enabled; }

  /** @brief Handle a surge event for the trigger i.e.; A positive or a negative surge. */
  bool on_surge(const typename T::SharedPtr msg)
  {
    if (!is_enabled()) return false;

    if (use_msg_stamp_ && !type_traits::HasHeader<T>::value)
    {
      RCLCPP_WARN_THROTTLE(
        *logger_, *clock_, 10000,
        "Trigger: %s is configured to use timestamps from the header but the underlying "
        "message-type(%s) is headerless. Using the clock time on this trigger!!",
        get_name().c_str(), get_msg_type().c_str());
    }

    auto stamp = get_time_stamp<T>(msg);
    auto trigger_duration = last_stamp_ - first_stamp_;
    bool negative_edge = false;

    if (msg && is_triggered(msg))
    {
      if (first_stamp_ == 0)
      {
        RCLCPP_DEBUG(*logger_, "%s positive edge", get_name().c_str());
        first_stamp_ = stamp.nanoseconds();
      }

      last_stamp_ = stamp.nanoseconds();
    } else if (trigger_duration >= static_cast<unsigned long>(persistance_duration_.nanoseconds()))
    {
      RCLCPP_DEBUG(*logger_, "%s negative edge", get_name().c_str());
      RCLCPP_INFO(
        *logger_, "%s Last Persistent trigger duration: %f seconds", get_name().c_str(),
        rclcpp::Duration(std::chrono::nanoseconds(trigger_duration)).seconds());
      trigger_pulses_.push_back({first_stamp_, last_stamp_});
      negative_edge = true;
      first_stamp_ = 0;
      last_stamp_ = 0;
    } else
    {
      first_stamp_ = 0;
      last_stamp_ = 0;
    }
    return negative_edge;
  }

protected:
  // The interface dictates that derived classes should implement this method to load the additional trigger config from the yaml config file.
  // This config (i.e. conditional parameters) may or may not be required depending on the triggering condition designed by the derived class.
  // Nevertheless, it is enforced so that the initialization of the "Conditional parameters" is always delegated to the derived class.
  virtual void configure_conditional_params(const YAML::Node & node) = 0;

private:
  // Utility function: Return message timestamp if message has header
  template <typename MsgT = T>
  typename std::enable_if<type_traits::HasHeader<MsgT>::value, rclcpp::Time>::type get_time_stamp(
    const typename T::SharedPtr msg)
  {
    if (msg && use_msg_stamp_)
    {
      return rclcpp::Time(msg->header.stamp);
    }
    return clock_->now();
  }

  // Utility function: Return current time if message has no header
  template <typename MsgT = T>
  typename std::enable_if<!type_traits::HasHeader<MsgT>::value, rclcpp::Time>::type get_time_stamp(
    const typename T::SharedPtr)
  {
    return clock_->now();
  }

  bool enabled_{false};
  uint64_t first_stamp_{0};
  uint64_t last_stamp_{0};
  std::vector<TriggerPulse> trigger_pulses_{};
  rclcpp::Duration persistance_duration_{rclcpp::Duration::from_seconds(0)};
  rclcpp::Clock::SharedPtr clock_{nullptr};
  std::shared_ptr<rclcpp::Logger> logger_{nullptr};
  bool use_msg_stamp_{false};
};

}  // namespace ros2bag_triggered

#endif  // TRIGGER_BASE_HPP
