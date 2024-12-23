#include <ros2bag_triggered/triggered_writer.hpp>
#include <chrono>

using std::chrono::duration_cast;

namespace ros2bag_triggered
{
    void TriggeredWriter::set_crop_points(int64_t start_time, int64_t end_time)
    {
        auto bag_start_time = duration_cast<std::chrono::nanoseconds>(metadata_.starting_time.time_since_epoch());
        auto bag_end_time = bag_start_time + duration_cast<std::chrono::seconds>(metadata_.duration);

        // Clip the crop-range so that it doesn't exceed the recorded bag's time-range.
        if(storage_options_.start_time_ns >= 0)
        {
            storage_options_.start_time_ns = std::max(start_time, bag_start_time.count());
        }
        storage_options_.end_time_ns = std::min(end_time, bag_end_time.count());

    }
}