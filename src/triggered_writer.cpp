#include <ros2bag_triggered/triggered_writer.hpp>
#include <chrono>

using std::chrono::duration_cast;

namespace ros2bag_triggered
{
    void TriggeredWriter::set_crop_points(int64_t start_time, int64_t end_time)
    {   

        start_time -= static_cast<int64_t>(config_.crop_gap);
        end_time += static_cast<int64_t>(config_.crop_gap);
        auto bag_start_time = duration_cast<std::chrono::nanoseconds>(metadata_.starting_time.time_since_epoch());
        auto bag_end_time = bag_start_time + duration_cast<std::chrono::seconds>(metadata_.duration);

        // Clip the crop-range so that it doesn't exceed the recorded bag's time-range.
        if(storage_options_.start_time_ns >= 0)
        {
            storage_options_.start_time_ns = std::max(start_time, bag_start_time.count());
        }
        storage_options_.end_time_ns = std::min(end_time, bag_end_time.count());

    }

    void TriggeredWriter::initialize(const std::optional<TriggeredWriter::Config>& config)
    {
        auto prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
        
        if (config.has_value())
        {
            config_ = config.value();
            return;
        }

        YAML::Node writer_cfg = YAML::LoadFile(prefix_path + "/config/parameters.yaml");
        YAML::convert<rosbag2_storage::StorageOptions>::decode(writer_cfg, storage_options_);
        config_.trigger_buffer_duration = writer_cfg["trigger_buffer_duration"].as<double>();
        config_.crop_gap = writer_cfg["crop_gap"].as<double>();
        config_.bag_root_dir = writer_cfg["bag_root_dir"].as<std::string>();
    }  
}