#ifndef TRIGGERED_WRITER_HPP
#define TRIGGERED_WRITER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <std_msgs/msg/bool.hpp>

namespace ros2bag_triggered
{

class TriggeredWriter : public rosbag2_cpp::writers::SequentialWriter
{
public:
    TriggeredWriter() : rosbag2_cpp::writers::SequentialWriter() {}
    ~TriggeredWriter() = default;

    /**
     * @brief Set the cropping time range of the bag file.
     * @param start_time The start time of the crop range in seconds.
     * @param end_time The end time of the crop range in seconds.
     */
    void set_crop_points(int64_t start_time, int64_t end_time);
};

} // namespace ros2bag_triggered

#endif // TRIGGERED_WRITER_HPP