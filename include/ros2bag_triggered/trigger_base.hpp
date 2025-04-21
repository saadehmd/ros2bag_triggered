#ifndef TRIGGER_BASE_HPP
#define TRIGGER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <ros2bag_triggered/type_traits.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros2bag_triggered {

template<typename T>
class TriggerBase
{
    static_assert(type_traits::IsRosIdlType<T>::value, "TriggerBase can only be instantiated with ROS2-IDL message types");

 public:
    using MsgType = T;
    explicit TriggerBase(double persistance_duration, const rclcpp::Clock::SharedPtr& clock, const std::shared_ptr<rclcpp::Logger> logger, bool use_msg_stamp)
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

    void fromYaml(const YAML::Node& node)
    {
        // YAML initialization shouldn't fallback on default constructed values and it should actively throw exceptions 
        // on problems parsing the configuration from yaml. Inclduing missing key-value pairs or wrong access-types.
        try
        {
            persistance_duration_ = rclcpp::Duration::from_seconds(node["persistance_duration"].as<double>());
            use_msg_stamp_ = node["use_msg_stamp"].as<bool>();
            //The base class is still responsible for calling the derived class's configuration of conditional parameters.
            //This is so that the derived class' implementation of yaml-configuration is minimal i.e.; The initialization 
            //of required parameters and exception-handling is all done in the base class.
            configureConditionalParams(node); 
        }
        catch (const YAML::BadConversion& e)
        {
            RCLCPP_ERROR(*logger_, "Bad-conversion while configuring %s: %s", getName().c_str(), e.what());
            throw;
        }
        catch(const YAML::InvalidNode& e)
        {
            RCLCPP_ERROR(*logger_, "Invalid-node while configuring %s: %s", getName().c_str(), e.what());
            throw;
        }
    }

    TriggerBase() = delete;
    virtual ~TriggerBase() = default;

    virtual bool isTriggered(const typename T::SharedPtr msg) const = 0;
    virtual std::string getName() const = 0;
    std::string getMsgType() const
    {
        return rosidl_generator_traits::name<T>();
    }

    virtual std::string getTriggerInfo() const
    {
        return "\n\tTrigger Type: " + getName() + 
               "\n\tMsg Type: " + getMsgType() + 
               "\n\tPersistance Duration: " + std::to_string(persistance_duration_.seconds()) + " seconds" +
               "\n\tUsing Message Stamps: " + (use_msg_stamp_ ? "true" : "false") +
               "\n\tEnabled: " + (enabled_ ? "true" : "false") +
               "\n\t=======================================================================================\n";
    }

    std::string getTriggerStats() const
    {
        std::string stats = "Trigger Stats for " + getName() + ":\n";
        for (const auto& trigger : all_triggers_)
        {
            auto seconds = std::to_string(rclcpp::Duration(std::chrono::nanoseconds(trigger.second - trigger.first)).seconds());
            stats += "\n\tTriggered from: " + std::to_string(trigger.first) + " to " + std::to_string(trigger.second) + " [" + seconds + " sec.s]\n";
        }
        stats += "\n\t=======================================================================================\n";
        return stats;
    }

    bool onSurgeSerialized(const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) 
    {
        if(!isEnabled()) return false;
        if(!serialized_msg)
        {
            RCLCPP_DEBUG(*logger_, "Null serialized msg. Abort signal recieved %s", getName().c_str());
            return onSurge(nullptr);
        }
        auto msg = std::make_shared<T>(); 
        rclcpp::Serialization<T> serializer;
        serializer.deserialize_message(serialized_msg.get(), msg.get());

        if (!msg) RCLCPP_ERROR(*logger_, "Deserialization resulted in null msg %s", getName().c_str());
        return onSurge(msg);
    }

    std::vector<std::pair<uint64_t, uint64_t>> getAllTriggers() const
    {
        return all_triggers_;
    }

    void reset()
    {
        first_stamp_ = 0;
        last_stamp_ = 0;
        all_triggers_.clear();
    }

    void setClock(const rclcpp::Clock::SharedPtr& clock)
    {
        clock_ = clock ; 
    }

    void setLogger(const std::shared_ptr<rclcpp::Logger>& logger)
    {
        logger_ = logger;
    }

    bool isUsingMsgStamps() const
    {
        return use_msg_stamp_;
    }

    bool isEnabled() const
    {
        return enabled_;
    }

    void setEnabled(bool enabled)
    {
        enabled_ = enabled;
    }

    bool onSurge(const typename T::SharedPtr msg)
    {   
        if(!isEnabled()) return false;

        if(use_msg_stamp_ && !type_traits::HasHeader<T>::value)
        {
            RCLCPP_WARN_THROTTLE(*logger_, *clock_, 10000, "Trigger: %s is configured to use timestamps from the header but the underlying message-type(%s) is headerless. Using the clock time on this trigger!!", getName().c_str(), getMsgType().c_str());
        }

        auto stamp = GetTimeStamp<T>(msg);
        auto trigger_duration = last_stamp_ - first_stamp_;
        bool negative_edge = false;
        
        if (msg && isTriggered(msg))
        {
            if (first_stamp_ == 0)
            {
                RCLCPP_DEBUG(*logger_, "%s positive edge", getName().c_str());
                first_stamp_ = stamp.nanoseconds();
            }

            last_stamp_ = stamp.nanoseconds();
        }
        else if (trigger_duration >= persistance_duration_.nanoseconds())
        {
            RCLCPP_DEBUG(*logger_, "%s negative edge", getName().c_str());
            RCLCPP_INFO(*logger_, "%s Last Persistent trigger duration: %f seconds", getName().c_str(), rclcpp::Duration(std::chrono::nanoseconds(trigger_duration)).seconds());
            all_triggers_.push_back(std::make_pair(first_stamp_, last_stamp_));
            negative_edge = true;
            first_stamp_ = 0;
            last_stamp_ = 0;
        }
        else
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
    virtual void configureConditionalParams(const YAML::Node& node) = 0;

    // Utility function: Return message timestamp if message has header
    template <typename MsgT = T>
    typename std::enable_if<type_traits::HasHeader<MsgT>::value, rclcpp::Time>::type
    GetTimeStamp(const typename T::SharedPtr msg) 
    {
        if (msg && use_msg_stamp_)
        {
            return rclcpp::Time(msg->header.stamp);
        }
        return clock_->now();
    }

    // Utility function: Return current time if message has no header
    template <typename MsgT = T>
    typename std::enable_if<!type_traits::HasHeader<MsgT>::value, rclcpp::Time>::type
    GetTimeStamp(const typename T::SharedPtr) 
    {
        return clock_->now();
    }

    bool enabled_{false};
    uint64_t first_stamp_{0};
    uint64_t last_stamp_{0};
    std::vector<std::pair<uint64_t, uint64_t>> all_triggers_{};
    rclcpp::Duration persistance_duration_{rclcpp::Duration::from_seconds(0)};
    rclcpp::Clock::SharedPtr clock_{nullptr};
    std::shared_ptr<rclcpp::Logger> logger_{nullptr};
    bool use_msg_stamp_{false};
};

} // namespace ros2bag_triggered

#endif // TRIGGER_BASE_HPP
