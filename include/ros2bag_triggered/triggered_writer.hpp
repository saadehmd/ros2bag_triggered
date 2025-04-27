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

using TriggerPulseMap = std::unordered_map<std::string, std::vector<std::pair<uint64_t, uint64_t>>>;

class TriggeredWriter : public rosbag2_cpp::writers::SequentialWriter
{
public:

    struct Config
    {
        std::string bag_root_dir{"./"};
        double trigger_buffer_duration{60.0};
        double crop_gap{5.0};
        bool bag_cropping{true};
        bool write_trigger_stats{true};
    };

    TriggeredWriter(const rclcpp::Logger& logger) 
    : rosbag2_cpp::writers::SequentialWriter(),
      logger_(logger) {}
    ~TriggeredWriter() = default;

    void initialize(const std::filesystem::path& writer_config);
    void close();
    void open(const rosbag2_storage::StorageOptions& storage_options, const rosbag2_cpp::ConverterOptions& converter_options) override;
    void write_trigger_stats(const std::string& trigger_stats);
    void plot_triggers(const TriggerPulseMap& all_triggers);

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
        return config_.bag_root_dir + "/" + bag_name_;
    }

    std::string get_bag_name() const
    {
        return bag_name_;
    }

    bool is_open() const
    {
        return storage_ != nullptr;
    }
    size_t get_topic_count() const
    {
        return topics_names_to_info_.size();
    }

protected:
    Config config_;
    std::string bag_name_;
    std::optional<rosbag2_storage::StorageOptions> rewrite_options_;
    bool is_triggered_;
    rclcpp::Logger logger_;

};

} // namespace ros2bag_triggered

#endif // TRIGGERED_WRITER_HPP