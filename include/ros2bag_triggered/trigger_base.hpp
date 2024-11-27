#ifndef TRIGGER_BASE_HPP
#define TRIGGER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <limits>

namespace ros2bag_triggered {
    template<typename T>
    class TriggerBase
    {
    public:
        using MessageType = T;

        TriggerBase(double persistance_duration)
        , persistance_duration_(rclcpp::Duration::from_seconds(persistance_duration)) 
        {}

        virtual ~TriggerBase() = default;
        virtual bool isTriggered(const typename T::SharedPtr msg) = 0;

        void onSurge(const typename T::SharedPtr msg, bool& negative_edge, bool use_stamp)
        {
            auto stamp = use_stamp ? rclcpp::Time(msg->header.stamp) : get_clock()->now();
            auto trigger_duration = last_stamp_ - first_stamp_;

            if(isTriggered(msg))
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
            else if(trigger_duration >= persistance_duration_.nanoseconds() + std::numeric_limits<uint64_t>::epsilon())
            {
                all_trigger_durations_.push_back(last_stamp_ - first_stamp_);
                negative_edge = true;
                first_stamp_ = 0;
                last_stamp_ = 0;
            }
            else
            {
                first_stamp_ = 0;
                last_stamp_ = 0;
            }

        }
        
    protected:
        
        uint64_t first_stamp_{0};
        uint64_t last_stamp_{0};
        std::vector<uint64_t> all_trigger_durations_{};
        rclcpp::Duration persistance_duration_{0};

    };

} // namespace ros2bag_triggered

#endif // TRIGGER_BASE_HPP