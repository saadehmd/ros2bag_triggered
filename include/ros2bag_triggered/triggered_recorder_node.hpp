#ifndef TRIGGERED_RECORDER_NODE_HPP
#define TRIGGERED_RECORDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <ros2bag_triggered/triggered_writer.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <memory>
#include <chrono>
#include <variant>
#include <unordered_map>
#include <yaml-cpp/yaml.h>


namespace ros2bag_triggered
{

template<typename TriggerVariant>
class TriggeredRecorderNode : public rclcpp::Node
{
public:
    TriggeredRecorderNode(std::string&& node_name = "triggered_recorder_node", 
                          const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    ~TriggeredRecorderNode();


private:

    struct Config
    {
        std::string bag_root_dir;
        std::string topic_config_path;
        uint64_t max_bagfile_duration;
        uint64_t max_bagfile_size;
        uint64_t max_cache_size;
        double trigger_buffer_duration;
        double crop_gap;
    };
    
    void initialize(const std::optional<Config>& config);
    //@ToDo: Replace the following with appropriate converter wrappers from rosbag2 API.
    void initialize_config(const std::optional<Config>& config);
    void reset_writer();
    void crop_and_reset();
    void create_subscriptions();
    void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg, 
                        const std::string& topic_name, 
                        const std::string& topic_type,
                        TriggerVariant& trigger);
    void crop_points_from_triggers(const std::variant<TriggerVariant>& trigger,
                                   const std::shared_ptr<rclcpp::SerializedMessage>& msg);

    template<std::size_t... Is>
    void initialize_triggers(std::index_sequence<Is...>);

    rosbag2_cpp::Writer writer_{/*writer_impl=*/std::make_unique<ros2bag_triggered::TriggeredWriter>()};
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    std::unordered_map<std::string, TriggerVariant> triggers_;
    YAML::Node topics_config_;
    Config config_;
    rclcpp::TimerBase::SharedPtr trigger_buffer_timer_;
    rosbag2_storage::StorageOptions last_bag_options_;
    std::pair<int64_t, int64_t> crop_points_{{-1, -1}};
};

} // namespace ros2bag_triggered

#endif // TRIGGERED_RECORDER_NODE_HPP