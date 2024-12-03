#include <ros2bag_triggered/triggered_recorder_node.hpp>

using namespace ros2bag_triggered;
using namespace rclcpp;

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::initialize()
{
    auto prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    bag_config_.bag_path = declare_parameter("bag_path", prefix_path + "/bags/");
    bag_config_.topic_config_path= declare_parameter("topic_config_path", prefix_path + "config/topic_config.yaml");
    bag_config_.bag_size = declare_parameter("bag_size", 10e10);

    YAML::Node topics_config;
    try
    {
        topics_config = YAML::LoadFile(bag_config_.topic_config_path+"/topic_config.yaml");
    }
    catch(YAML::BadFile&)
    {
        RCLCPP_ERROR(get_logger(), "The topic-config path on param-file is invalid");
        throw YAML::BadFile();
    }
}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::add_to_subscriptions(const YAML::Node& topics_cfg)
{
    for (const auto& topic : topics_cfg)
    {
        auto topic_name = topic.first.as<std::string>();
        auto topic_cfg  = topic.second;
        auto msg_type = topic_cfg["msg_type"].as<std::string>();

        if(topic_cfg["triggers"].IsDefined())
        {   

            for (const auto& trigger_info : topic_cfg["triggers"])
            {
                auto trigger_type = trigger_info["type"].as<std::string>();
                auto persist_duration = trigger_info["persist_duration"].as<double>();
                auto& trigger = triggers_[trigger_type];
                using TriggerType = std::decay_t<decltype(trigger)>;
                std::get<TriggerType>(trigger).emplace(persist_duration, get_clock);
            }
        }
        /*std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback = 
          std::bind(&RecorderNode::topic_callback, this, std::placeholders::_1, topic_name, topic_type, trigger);*/
    }

}

template<typename TriggerVariant>
template<std::size_t... Is>
void  TriggeredRecorderNode<TriggerVariant>::initialize_triggers(std::index_sequence<Is...>)
{
    ((triggers_[std::variant_alternative_t<Is, TriggerVariant>::name] = std::variant_alternative_t<Is, TriggerVariant>{}), ...);
}

template<typename TriggerVariant>
TriggeredRecorderNode<TriggerVariant>::TriggeredRecorderNode(std::string&& node_name, 
                          const rclcpp::NodeOptions& node_options)
: rclcpp::Node(std::move(node_name), node_options) 
{
    initialize_triggers(std::make_index_sequence<std::variant_size<TriggerVariant>::value>{});
    initialize();
}
