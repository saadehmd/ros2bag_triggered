#include <ros2bag_triggered/triggered_writer.hpp>
#include <chrono>

susing std::chrono::duration_cast;

namespace ros2bag_triggered
{
    TriggeredWriter::set_crop_points(int64_t start_time, int64_t end_time)
    {
        auto bag_start_time = duration_cast<std::chrono::nanoseconds>(metadata_.start_time.time_since_epoch());
        auto bag_end_time = bag_start_time + duration_cast<std::chrono::seconds>(bag_metadata->duration);

        // Clip the crop-range so that it doesn't exceed the recorded bag's time-range.
        if(storage_options_.start_time_ns >= 0)
        {
            storage_options_.start_time_ns = std::max(crop_points_.first, bag_start_time.count()) 
        }
        storage_options_.end_time_ns = std::min(crop_points_.second, bag_end_time.count());

    }
}