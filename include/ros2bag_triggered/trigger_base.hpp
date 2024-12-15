#ifndef TRIGGER_BASE_HPP
#define TRIGGER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <limits>

namespace ros2bag_triggered {

template<typename T>
class TriggerBase
{
public:

    explicit TriggerBase(double persistance_duration, const rclcpp::Clock::SharedPtr& clock, bool use_msg_stamp)
    : persistance_duration_(rclcpp::Duration::from_seconds(persistance_duration)),
      use_msg_stamp_(use_msg_stamp),
      clock_(clock) 
    {}

    TriggerBase() = delete;
    virtual ~TriggerBase() = default;
    virtual bool isTriggered(const typename T::SharedPtr msg) = 0;

    bool onSurge(const typename T::SharedPtr msg)
    {
        if (!use_msg_stamp_ && !clock_)
        {
            throw std::runtime_error("No stamps on the msgs and no clock provided");
        }

        auto stamp = use_msg_stamp_ ? rclcpp::Time(msg->header.stamp) : clock_->now();
        auto trigger_duration = last_stamp_ - first_stamp_;
        bool negative_edge = false;

        if (isTriggered(msg))
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
        else if (trigger_duration >= persistance_duration_.nanoseconds() + std::numeric_limits<uint64_t>::epsilon())
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
    
protected:
    uint64_t first_stamp_{0};
    uint64_t last_stamp_{0};
    bool use_msg_stamp_{false};
    std::vector<std::pair<uint64_t, uint64_t>> all_triggers_{};
    rclcpp::Duration persistance_duration_{0};
    rclcpp::Clock::SharedPtr clock_{nullptr};
};

} // namespace ros2bag_triggered

#endif // TRIGGER_BASE_HPP
