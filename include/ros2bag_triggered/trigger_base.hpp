#ifndef TRIGGER_BASE_HPP
#define TRIGGER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <ros2bag_triggered/type_traits.hpp>

namespace ros2bag_triggered {

template<typename T>
class TriggerBase
{
    static_assert(type_traits::IsRosIdlType<T>::value, "TriggerBase can only be instantiated with ROS2-IDL message types");

public:

    explicit TriggerBase(double persistance_duration, const rclcpp::Clock::SharedPtr& clock, bool use_msg_stamp)
    : persistance_duration_(rclcpp::Duration::from_seconds(persistance_duration)),
      use_msg_stamp_(use_msg_stamp),
      clock_(clock) 
    {}

    TriggerBase() = default;
    virtual ~TriggerBase() = default;
    virtual bool isTriggered(const typename T::SharedPtr msg) const = 0;
    virtual std::string getName() const = 0;    

    bool onSurgeSerialized(const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) 
    {
        if(!isEnabled()) return false;
        if(!serialized_msg)
        {
            return onSurge(nullptr);
        }
        typename T::SharedPtr msg; 
        rclcpp::Serialization<T> serializer;
        serializer.deserialize_message(serialized_msg.get(), msg.get());
        return onSurge(msg);
    }

    bool onSurge(const typename T::SharedPtr msg)
    {   

        if(!isEnabled()) return false;

        if (!use_msg_stamp_ && !clock_)
        {
            throw std::runtime_error("No stamps on the msgs and no clock provided");
        }
        auto stamp = use_msg_stamp_ && msg ? GetTimeStamp<T>(msg) : clock_->now();
        auto trigger_duration = stamp.nanoseconds() - first_stamp_;
        bool negative_edge = false;

        if (msg && isTriggered(msg))
        {
            if (first_stamp_ == 0)
            {
                first_stamp_ = stamp.nanoseconds();
            }
            else    
            {
                last_stamp_ = stamp.nanoseconds();
            }
        }
        else if (trigger_duration >= persistance_duration_.nanoseconds())
        {
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

    bool isUsingMsgStamps() 
    {
        return use_msg_stamp_;
    }

    bool isEnabled()
    {
        return enabled_ && persistance_duration_.nanoseconds() > 0;
    }

    void setEnabled(bool enabled)
    {
        enabled_ = enabled;
    }
    
protected:

    // The interface dictates that derived classes should implement this method to load the trigger config from a YAML node.
    // The method is protected and only accessible from the dedicated YAML-based constructor, to protect the encapsulation.
    virtual void fromYaml(const YAML::Node& node) = 0;

    // Utility function: Return message timestamp if message has header
    template <typename MsgT = T>
    typename std::enable_if<type_traits::HasHeader<MsgT>::value, rclcpp::Time>::type
    GetTimeStamp(const typename T::SharedPtr msg) 
    {
        return rclcpp::Time(msg->header.stamp);
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
    bool use_msg_stamp_{false};
    std::vector<std::pair<uint64_t, uint64_t>> all_triggers_{};
    rclcpp::Duration persistance_duration_{rclcpp::Duration::from_seconds(0)};
    rclcpp::Clock::SharedPtr clock_{nullptr};
};

} // namespace ros2bag_triggered

#endif // TRIGGER_BASE_HPP
