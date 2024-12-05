#ifndef TRIGGERED_RECORDER_NODE_HPP
#define TRIGGERED_RECORDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <ros2bag_triggered/trigger_base.hpp>
#include <memory>
#include <variant>
#include <utility>
#include <unordered_map>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <yaml-cpp/yaml.h>

namespace ros2bag_triggered
{

template<typename TriggerVariant>
class TriggeredRecorderNode : public rclcpp::Node
{
public:
    TriggeredRecorderNode(std::string&& node_name, 
                          const rclcpp::NodeOptions& node_options);
    ~TriggeredRecorderNode();


private:

    struct BagConfig
    {
        std::string bag_path;
        std::string topic_config_path;
        uint64_t bag_size;
    };
    
    
    void initialize();
    void reset_writer();
    void create_subscriptions(const YAML::Node& topics_cfg);
    void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg, 
                        const std::string& topic_name, 
                        const std::string& topic_type,
                        TriggerVariant& trigger);

    template<std::size_t... Is>
    void initialize_triggers(std::index_sequence<Is...>);

    rosbag2_cpp::Writer writer_;
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    std::unordered_map<std::string, TriggerVariant> triggers_;
    BagConfig bag_config_;
    
};

} // namespace ros2bag_triggered

#endif // TRIGGERED_RECORDER_NODE_HPP