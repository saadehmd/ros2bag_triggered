#ifndef TRIGGERED_RECORDER_NODE_HPP
#define TRIGGERED_RECORDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <ros2bag_triggered/triggered_writer.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ros2bag_triggered/triggered_recorder_node.hpp>
#include <ros2bag_triggered/trigger_variant_visitors.hpp>

#include <memory>
#include <chrono>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>

namespace ros2bag_triggered
{


template<typename TriggerVariant>
class TriggeredRecorderNode : public rclcpp::Node
{
public:
    TriggeredRecorderNode(std::string&& node_name = "triggered_recorder_node", 
                          const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : rclcpp::Node(std::move(node_name), node_options) 
    {
        initialize(std::nullopt);
    }

    ~TriggeredRecorderNode()
    {
        reset();
    }

private:

    void initialize(const std::optional<ros2bag_triggered::TriggeredWriter::Config>& config)
    {
        RCLCPP_INFO(get_logger(), "Initializing Triggered Recorder Node");
        auto prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
        try
        {
            topics_config_ = YAML::LoadFile(prefix_path + "/config/topic_config.yaml");
            get_writer_impl().initialize(config);
        }
        catch(YAML::BadFile& exc)
        {
            RCLCPP_ERROR(get_logger(), "The topic-config path on param-file is invalid");
            throw exc;
        }
        catch(YAML::Exception& exc)
        {
            RCLCPP_ERROR(get_logger(), "Parsing error in parsing the topic/writer configs.");
            throw exc;
        }

        initialize_triggers(std::make_index_sequence<std::variant_size<TriggerVariant>::value>{});
        reset_writer();

        // This is an expensive operation, so we do it only once in the beginning.
        create_subscriptions();

        trigger_buffer_timer_ = create_wall_timer(
            std::chrono::duration<double>(get_writer_impl().get_config().trigger_buffer_duration), 
            [this]() 
            { 
                reset();
                //TODO: Test whether the 'create_topic()' calls are necessary after the bag is closed and reopened with new storage options.
                reset_writer(); 
            }
        );

    } 

    void reset_writer()
    {   
        RCLCPP_INFO(get_logger(), "Resetting the writer...");
        rosbag2_storage::StorageOptions storage_options;
        auto stamp = get_clock()->now().nanoseconds();
        writer_.open(std::to_string(stamp));
        
        //Create all topics immediately after creating the new writer
        for (const auto& topic : topics_config_)
        {
            auto topic_name = topic.first.as<std::string>();
            auto topic_cfg  = topic.second;
            auto msg_type = topic_cfg["msg_type"].as<std::string>();
            auto is_recorded = topic_cfg["record"] ? topic_cfg["record"].as<bool>() : true;

            if(!is_recorded)
            {
                continue;
            }
            std::string serialization_format = "cdr";
            auto topic_meta = rosbag2_storage::TopicMetadata();
            topic_meta.name = topic_name;
            topic_meta.type = msg_type;
            topic_meta.serialization_format = serialization_format;
            writer_.create_topic(topic_meta);
        }
    }

    void reset()
    {   
        auto& triggered_writer = get_writer_impl();
        auto base_folder = triggered_writer.get_base_folder();
        std::string trigger_stats;
        bool write_trigger_stats = triggered_writer.get_config().write_trigger_stats;

        /* Send an early termination surge to all triggers to
        check if any of them are activated and can update
        crop points for the bag. Also get trigger stats for writing later. */
        for (auto& trigger : triggers_)
        {   
            crop_points_from_triggers(trigger.second, /*msg=*/nullptr); // nullptr acts as an abort signal to the triggers.
            if (write_trigger_stats)
            {
                trigger_stats += std::visit(getTriggerStats, trigger.second);
            }
            std::visit(resetTrigger, trigger.second);
        }

        triggered_writer.set_crop_points(crop_points_.first, crop_points_.second);
        crop_points_ = std::make_pair(-1, -1); //Reset crop points for the next bag.
        writer_.close();
        triggered_writer.write_trigger_stats(trigger_stats);   
    }

    void create_subscriptions()
    {
        for (const auto& topic : topics_config_)
        {
            auto topic_name = topic.first.as<std::string>();
            auto topic_cfg  = topic.second;
            auto msg_type = topic_cfg["msg_type"].as<std::string>();
            auto is_recorded = topic_cfg["record"] ? topic_cfg["record"].as<bool>() : true;
            
            std::string triggers_debug_info = "Subscribed to topic: " + topic_name + " Msg type: " +  msg_type + " with triggers: [";
            std::vector<std::reference_wrapper<TriggerVariant>> triggers;

            if(topic_cfg["triggers"].IsDefined())
            {   
                
                for (const YAML::Node& triggers_info : topic_cfg["triggers"])
                {
                    auto trigger_type = triggers_info["type"].as<std::string>();

                    try
                    {
                        auto& trigger = triggers_.at(trigger_type);
                        std::visit([&triggers_info](auto& arg){configureTrigger(arg, triggers_info);}, trigger);                     
                        triggers_debug_info += std::visit(getTriggerInfo, trigger);
                        triggers.push_back(trigger);
                    }
                    catch(const std::out_of_range& e)
                    {
                        RCLCPP_ERROR(get_logger(), "Trigger type %s not found.", trigger_type.c_str());
                        RCLCPP_ERROR(get_logger(), "Registered Trigger types are:-");
                        for(auto& [key, value] : triggers_)
                        {
                            RCLCPP_ERROR(get_logger(), "- %s", key.c_str());
                        }
                        throw e;
                    }
                    catch(const std::runtime_error& e)
                    {
                        RCLCPP_ERROR(get_logger(), e.what());
                        throw e;
                    }
                }
            }

            if(!is_recorded && triggers.empty())
            {
                RCLCPP_WARN(get_logger(), "Topic %s is not recorded and has no triggers. Skipping...", topic_name.c_str());
                continue;
            }
            std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback = 
                        std::bind(&TriggeredRecorderNode::topic_callback, this, std::placeholders::_1, topic_name, msg_type, triggers);            
            subscriptions_.push_back(create_generic_subscription(topic_name, msg_type, rclcpp::QoS(10), callback));
            RCLCPP_INFO(get_logger(), (triggers_debug_info + "]").c_str());                
        }
    }

    void topic_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                        const std::string& topic_name, 
                        const std::string& topic_type,
                        const std::vector<std::reference_wrapper<TriggerVariant>>& triggers)
    {  
        rclcpp::Time time_stamp = get_clock()->now();
        writer_.write(*msg, topic_name, topic_type, time_stamp);

        for (auto& trigger : triggers)
        {
            crop_points_from_triggers(trigger, msg);
        }
    }

    void crop_points_from_triggers(TriggerVariant& trigger, const std::shared_ptr<rclcpp::SerializedMessage>& msg)
    {
        bool negative_edge = std::visit([&msg](auto& arg){return onSurge(arg, msg);}, trigger);
        if (negative_edge)
        {
            auto all_triggers = std::visit(getAllTriggers, trigger);
            if (all_triggers.empty())
            {
                return;
            }

            auto& last_trigger = all_triggers.back();
            auto start_point = static_cast<int64_t>(last_trigger.first);
            auto stop_point =  static_cast<int64_t>(last_trigger.second);
            crop_points_.first = crop_points_.first >= 0  ? std::min(crop_points_.first, start_point) : last_trigger.first;
            crop_points_.second = std::max(crop_points_.second, stop_point);
            
        }
    }

    template<std::size_t... Is>
    void initialize_triggers(std::index_sequence<Is...>)
    {   
        (([&]() {
            using TriggerT = std::variant_alternative_t<Is, TriggerVariant>;
            if constexpr (!std::is_same_v<TriggerT, std::monostate>)
            {
                auto temp = TriggerT{/*persistance_duration=*/0, get_clock(), std::make_shared<rclcpp::Logger>(get_logger()), /*use_msg_stamp=*/true};
                triggers_[temp.getName()] = std::move(temp);
            }         
        }()), ...);

        std::string triggers_debug_info = "Recorder Node initialized with following trigger types:-";
        for(auto& [key, value] : triggers_)
        {
            const auto msg_type = std::visit(getMsgType, value);
            triggers_debug_info += "\n\tTrigger Type: " + key + ",\tMsg Type: " + msg_type;
        }
        RCLCPP_INFO(get_logger(), triggers_debug_info.c_str());
    }
    
    ros2bag_triggered::TriggeredWriter & get_writer_impl()
    {
        auto& base_writer = writer_.get_implementation_handle();
        auto& triggered_writer = static_cast<ros2bag_triggered::TriggeredWriter&>(base_writer);
        return triggered_writer;
    }

    rosbag2_cpp::Writer writer_{/*writer_impl=*/std::make_unique<ros2bag_triggered::TriggeredWriter>(get_logger())};
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    std::unordered_map<std::string, TriggerVariant> triggers_;
    YAML::Node topics_config_;
    rclcpp::TimerBase::SharedPtr trigger_buffer_timer_;
    std::pair<int64_t, int64_t> crop_points_{-1, -1};
};

} // namespace ros2bag_triggered

#endif // TRIGGERED_RECORDER_NODE_HPP