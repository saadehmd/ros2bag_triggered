#ifndef TRIGGERED_WRITER_HPP
#define TRIGGERED_WRITER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rosbag2_storage/default_storage_id.hpp>
#include <std_msgs/msg/bool.hpp>
#include <filesystem>

namespace ros2bag_triggered
{

class TriggeredWriter : public rosbag2_cpp::writers::SequentialWriter
{
public:

    struct Config
    {
        std::string bag_root_dir;
        double trigger_buffer_duration;
        double crop_gap;
        bool write_trigger_stats;
    };

    TriggeredWriter() : rosbag2_cpp::writers::SequentialWriter() {}
    ~TriggeredWriter() = default;

    void initialize(const std::optional<Config>& writer_config);
    void close();
    void open(const rosbag2_storage::StorageOptions& storage_options, const rosbag2_cpp::ConverterOptions& converter_options);

    /**
     * @brief Set the cropping time range of the bag file.
     * @param start_time The start time of the crop range in seconds.
     * @param end_time The end time of the crop range in seconds.
     */
    void set_crop_points(int64_t start_time, int64_t end_time);

    Config get_config() const
    {
        return config_;
    }
    
    rosbag2_storage::StorageOptions get_storage_options() const
    {
        // This should only return a copy of the storage options, otherwise it violates the encapsulation.
        return storage_options_;
    }

    std::string get_base_folder() const
    {
        return base_folder_;
    }

private:
    Config config_;
    std::string base_folder_;
    std::optional<rosbag2_storage::StorageOptions> rewrite_options_;

};

} // namespace ros2bag_triggered

#endif // TRIGGERED_WRITER_HPP