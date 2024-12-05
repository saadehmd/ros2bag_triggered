#include <ros2bag_triggered/triggered_recorder_node.hpp>

using namespace ros2bag_triggered;
using namespace rclcpp;

template<typename TriggerVariant>
TriggeredRecorderNode<TriggerVariant>::TriggeredRecorderNode(std::string&& node_name, 
                          const rclcpp::NodeOptions& node_options)
: rclcpp::Node(std::move(node_name), node_options) 
{
    initialize();
}

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

    initialize_triggers(std::make_index_sequence<std::variant_size<TriggerVariant>::value>{});
    reset_writer();
    create_subscriptions(topics_config);

}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::reset_writer()
{
    rosbag2_storage::StorageOptions storage_options;
    auto stamp = get_clock()->now().nanoseconds();
    storage_options.uri = bag_config_.bag_path + '/' + std::to_string(stamp);
    storage_options.max_bagfile_size = bag_config_.bag_size;
    storage_options.storage_id = "sqlite3";
    writer_.open(storage_options);
}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::create_subscriptions(const YAML::Node& topics_cfg)
{
    for (const auto& topic : topics_cfg)
    {
        auto topic_name = topic.first.as<std::string>();
        auto topic_cfg  = topic.second;
        auto msg_type = topic_cfg["msg_type"].as<std::string>();

        TriggerVariant trigger_initialized; // should be std::monostate by default and the other types should not be default constructible
        if(topic_cfg["triggers"].IsDefined())
        {   
            for (const auto& trigger_info : topic_cfg["triggers"])
            {
                auto trigger_type = trigger_info["type"].as<std::string>();
                auto persist_duration = trigger_info["persist_duration"].as<double>();
                auto& trigger = triggers_[trigger_type];
                trigger_initialized = std::get<decltype(triggers_[trigger])>(trigger).emplace(persist_duration, get_clock);
            }
        }
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback = 
          std::bind(&TriggeredRecorderNode::topic_callback, this, std::placeholders::_1, topic_name, msg_type, trigger_initialized);

        subscriptions_.push_back(create_generic_subscription(topic_name, msg_type, rclcpp::QoS(10), callback));
    }

}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                                           const std::string& topic_name, 
                                                           const std::string& topic_type,
                                                           TriggerVariant& trigger)
{  
    rclcpp::Time time_stamp = this->now();
    writer_.write(*msg, topic_name, topic_type, time_stamp);

    if (trigger.index() > 0) 
    {
        bool negative_edge = trigger.onSurge(msg);
    }
  
}

template<typename TriggerVariant>
template<std::size_t... Is>
void  TriggeredRecorderNode<TriggerVariant>::initialize_triggers(std::index_sequence<Is...>)
{
    ((triggers_[std::variant_alternative_t<Is, TriggerVariant>::name] = std::variant_alternative_t<Is, TriggerVariant>{}), ...);
}
