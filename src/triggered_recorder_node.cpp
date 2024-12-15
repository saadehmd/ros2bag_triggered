#include <ros2bag_triggered/triggered_recorder_node.hpp>
#include <rosbag2_transport/bag_rewrite.hpp>
#include <rosbag2_transport/record_options.hpp>
#include "rosbag2_storage/ros_helper.hpp"
#include <rosbag2_storage/default_storage_id.hpp>

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
void TriggeredRecorderNode<TriggerVariant>::initialize(const std::optional<Config>& config)
{
    initialize_config(config);
    trigger_buffer_timer_ = create_wall_timer(std::chrono::seconds(config_.trigger_buffer_duration), [this](){crop_and_reset();});
    YAML::Node topics_config;
    try
    {
        topics_config = YAML::LoadFile(config_.topic_config_path+"/topic_config.yaml");
    }
    catch(YAML::BadFile&)
    {
        RCLCPP_ERROR(get_logger(), "The topic-config path on param-file is invalid");
        throw YAML::BadFile();
    }

    initialize_triggers(std::make_index_sequence<std::variant_size<TriggerVariant>::value>{});
    reset_writer();

    // This is an expensive operation, so we do it only once in the beginning.
    create_subscriptions();

}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::initialize_config(const std::optional<Config>& config)
{
    auto prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    
    if (config.has_value())
    {
        config_ = config.value();
        return;
    }

    config_.bag_root_dir = declare_parameter("bag_path", prefix_path + "/ros2bag_triggered/");
    config_.topic_config_path = declare_parameter("topic_config_path", prefix_path + "config/topic_config.yaml");
    config_.max_bagfile_duration = declare_parameter("max_bagfile_duration", 300);
    config_.max_bagfile_size = declare_parameter("max_bagfile_size", 10e10);
    config_.max_cache_size = declare_parameter("max_cache_size", 0);
    config_.trigger_buffer_duration = declare_parameter("trigger_buffer_duration", 600);
}   

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::reset_writer()
{
    rosbag2_storage::StorageOptions storage_options;
    auto stamp = get_clock()->now().nanoseconds();
    last_bag_options_.uri = config_.bag_root_dir + '/' + std::to_string(stamp);
    last_bag_options_.max_bagfile_size = config_.max_bagfile_size;
    last_bag_options_.max_bagfile_duration = config_.max_bagfile_duration;
    last_bag_options_.max_cache_size = config_.max_cache_size;
    last_bag_options_.storage_id = rosbag2_storage::get_default_storage_id();
    writer_.open(last_bag_options_);

    //Create all topics immediately after creating the new writer
    for (const auto& topic : topics_config_)
    {
        auto topic_name = topic.first.as<std::string>();
        auto topic_cfg  = topic.second;
        auto msg_type = topic_cfg["msg_type"].as<std::string>();
        std::string serialization_format = "cdr";
        auto metadata = rosbag2_storage::TopicMetadata(topic_name, msg_type, serialization_format);
        writer_.create_topic(metadata);
    }
}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::crop_and_reset()
{
    if(crop_points_.first < 0)
    {
        RCLCPP_WARN(get_logger(), "No triggers were detected on the bag: %s", last_bag_options_.uri.c_str());
        return;
    }

    /* Write the bag metadata and close before re-writing. */
    writer_.close();

    /* Send an early termination surge to all triggers to
       check any of them are activated and can provide
       crop points for the bag */
    for (auto& trigger : triggers_)
    {
        if(trigger.index() > 0)
        {
            crop_points_from_triggers(trigger, /*msg=*/nullptr);
            trigger.reset();
        }
    }

    /* Crop is simply a rewrite with new start/end timestamp information.
       Only supported with ROS2 >= Jazzy  */
    rosbag2_storage::StorageOptions rewrite_bag_options = last_bag_options_;
    rewrite_bag_options.uri = last_bag_options_.uri + "_triggered";
    rewrite_bag_options.start_time_ns = crop_points_.first;
    rewrite_bag_options.end_time_ns = crop_points_.second;
    rosbag2_transport::RecordOptions record_options;
    record_options.all_topics = true;
    rosbag2_transport::bag_rewrite({last_bag_options_}, {std::make_pair(rewrite_bag_options, record_options)});

    reset_writer();
    
}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::create_subscriptions()
{
    for (const auto& topic : topics_config_)
    {
        auto topic_name = topic.first.as<std::string>();
        auto topic_cfg  = topic.second;
        auto msg_type = topic_cfg["msg_type"].as<std::string>();

        TriggerVariant trigger_initialized; // should be std::monostate by default and the other types should not be default constructible
        if(topic_cfg["triggers"].IsDefined())
        {   
            for (const YAML::Node& trigger_info : topic_cfg["triggers"])
            {
                auto trigger_type = trigger_info["type"].as<std::string>();
                auto& trigger = triggers_[trigger_type];
                trigger_initialized = std::get<decltype(triggers_[trigger])>(trigger).emplace(trigger_info);
                if (!trigger_initialized.isUsingMsgStamps())
                {
                    trigger_initialized.set_clock(get_clock());
                }
            }
        }
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback = 
          std::bind(&TriggeredRecorderNode::topic_callback, this, std::placeholders::_1, topic_name, msg_type, trigger_initialized);

        subscriptions_.push_back(create_generic_subscription(topic_name, msg_type, rclcpp::QoS(10), callback));
    }

}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::topic_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                                           const std::string& topic_name, 
                                                           const std::string& topic_type,
                                                           TriggerVariant& trigger)
{  
    rclcpp::Time time_stamp = this->now();
    writer_.write(*msg, topic_name, topic_type, time_stamp);

    if (trigger.index() > 0) // if initialized, otherwise the topic has no triggers on it.
    {
        crop_points_from_triggers(trigger, msg);
    }
  
}

template<typename TriggerVariant>
void TriggeredRecorderNode<TriggerVariant>::crop_points_from_triggers(const std::variant<TriggerVariant>& trigger,
                                                                      const std::shared_ptr<rclcpp::SerializedMessage>& msg)
{
    bool negative_edge = trigger.onSurge(msg);
    if (negative_edge)
    {
        auto last_trigger = trigger.getAllTriggers().back();
        crop_points_.first = crop_points_.first > 0  ? std::min(crop_points_.first, last_trigger.first) : last_trigger.first;
        crop_points_.second = std::max(crop_points_.second, last_trigger.second);
    }
}

template<typename TriggerVariant>
template<std::size_t... Is>
void  TriggeredRecorderNode<TriggerVariant>::initialize_triggers(std::index_sequence<Is...>)
{
    ((triggers_[std::variant_alternative_t<Is, TriggerVariant>::name] = std::variant_alternative_t<Is, TriggerVariant>{}), ...);
}
