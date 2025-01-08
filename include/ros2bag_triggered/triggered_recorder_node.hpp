#ifndef TRIGGERED_RECORDER_NODE_HPP
#define TRIGGERED_RECORDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <ros2bag_triggered/triggered_writer.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ros2bag_triggered/triggered_recorder_node.hpp>
#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <variant>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>


namespace ros2bag_triggered
{

//Visitors.
auto resetTrigger = [](auto& trigger) {trigger.reset();};
auto isUsingMsgStamps = [](auto& trigger) -> bool {return trigger.isUsingMsgStamps();};
auto setClock = [](auto& trigger, const rclcpp::Clock::SharedPtr& clock){trigger.setClock(clock);};
auto setLogger = [](auto& trigger, const std::shared_ptr<rclcpp::Logger>& logger){trigger.setLogger(logger);};
auto getAllTriggers = [](auto& trigger) -> std::vector<std::pair<uint64_t, uint64_t>> {return trigger.getAllTriggers();};
auto getMsgType = [](auto& trigger) -> std::string {return trigger.getMsgType();};
auto getTriggerInfo = [](auto& trigger) -> std::string {return trigger.getTriggerInfo();};
auto getTriggerStats = [](auto& trigger) -> std::string {return trigger.getTriggerStats();};
auto onSurge = [](auto& trigger, const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg) -> bool 
{   
    return trigger.onSurgeSerialized(serialized_msg);
};
auto configureTrigger = [](auto& trigger, const YAML::Node& config)
{   
    if (trigger.isEnabled())
    {   
        // @ToDo: Support registering triggers of the same type, to different topics.
        throw std::runtime_error("Trigger type: " + trigger.getName() + " is already registered to a topic. Registering single trigger-type to multiple topics is not yet supported.");
    }
    using TriggerT = typename std::remove_reference<decltype(trigger)>::type;
    trigger = TriggerT(config);
};


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

    ~TriggeredRecorderNode() = default;


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
            [this]() { crop_and_reset(); }
        );

    } 

    void reset_writer()
    {   
        RCLCPP_INFO(get_logger(), "Resetting the writer...");
        rosbag2_storage::StorageOptions storage_options;
        auto stamp = get_clock()->now().nanoseconds();
        auto& triggered_writer = get_writer_impl();
        auto last_storage_options = triggered_writer.get_storage_options();
        last_storage_options.uri = triggered_writer.get_config().bag_root_dir + '/' + std::to_string(stamp);

        writer_.open(last_storage_options);
        RCLCPP_INFO(get_logger(), "Opened new bag file: %s", last_storage_options.uri.c_str());

        //Create all topics immediately after creating the new writer
        for (const auto& topic : topics_config_)
        {
            auto topic_name = topic.first.as<std::string>();
            auto topic_cfg  = topic.second;
            auto msg_type = topic_cfg["msg_type"].as<std::string>();
            std::string serialization_format = "cdr";
            auto topic_meta = rosbag2_storage::TopicMetadata();
            topic_meta.name = topic_name;
            topic_meta.type = msg_type;
            topic_meta.serialization_format = serialization_format;
            writer_.create_topic(topic_meta);
        }
    }

    void crop_and_reset()
    {   
        /* Send an early termination surge to all triggers to
        check if any of them are activated and can update
        crop points for the bag */
        for (auto& trigger : triggers_)
        {   
            crop_points_from_triggers(trigger.second, /*msg=*/nullptr); // nullptr acts as an abort signal to the triggers.
            std::visit(resetTrigger, trigger.second);
        }

        auto& triggered_writer = get_writer_impl();
        if(crop_points_.first < 0 || crop_points_.second < 0)
        {
            RCLCPP_WARN(get_logger(), "No triggers were detected on the bag: %s", triggered_writer.get_storage_options().uri.c_str());
        }

        if (triggered_writer.get_config().write_trigger_stats)
        {
        
            std::ofstream stats_file(triggered_writer.get_config().bag_root_dir + "/trigger_stats.txt");
            if (!stats_file.is_open())
            {
                RCLCPP_ERROR(get_logger(), "Failed to open trigger stats file.");
                return;
            }

            for (const auto& trigger : triggers_)
            {
                stats_file << std::visit(getTriggerStats, trigger.second);
            }

            stats_file.close();
        }
        
        /* Change the start and end times of the bag before closing, to impliciltly perform cropping of the bag. 
        Only supported with ROS2 >= Jazzy. If there were no triggers, one or both crop points are negative and 
        hence the writer discards all the recorded messages during writer.close() call. */
        triggered_writer.set_crop_points(crop_points_.first, crop_points_.second);

        /** Closing checks the time-range of messages before writing and discards if messages are out of this range.*/
        writer_.close();

        // @todo: Remove the following after appropriate testing of above method.
        /* Crop is simply a rewrite with new start/end timestamp information.
        Only supported with ROS2 >= Jazzy  
        rosbag2_storage::StorageOptions rewrite_bag_options = last_bag_options_;
        rewrite_bag_options.uri = last_bag_options_.uri + "_triggered";
        rewrite_bag_options.start_time_ns = std::max(crop_points_.first, bag_start_time);
        rewrite_bag_options.end_time_ns = std::min(crop_points_.second, bag_end_time);
        rosbag2_transport::RecordOptions record_options;
        record_options.all_topics = true;
        rosbag2_transport::bag_rewrite({last_bag_options_}, {std::make_pair(rewrite_bag_options, record_options)});**/

        //TODO: Test whether the 'create_topic()' calls are necessary after the bag is closed and reopened with new storage options.
        reset_writer();
        
    }

    void create_subscriptions()
    {
        for (const auto& topic : topics_config_)
        {
            auto topic_name = topic.first.as<std::string>();
            auto topic_cfg  = topic.second;
            auto msg_type = topic_cfg["msg_type"].as<std::string>();
            
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
                        if(!std::visit(isUsingMsgStamps, trigger))
                        {
                            std::visit([&](auto& arg){setClock(arg, get_clock());}, trigger);
                        }
                        std::visit([&](auto& arg){setLogger(arg, std::make_shared<rclcpp::Logger>(get_logger()));}, trigger);
                        
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
            auto temp = TriggerT{};
            triggers_[temp.getName()] = std::move(temp);
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

    rosbag2_cpp::Writer writer_{/*writer_impl=*/std::make_unique<ros2bag_triggered::TriggeredWriter>()};
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    std::unordered_map<std::string, TriggerVariant> triggers_;
    YAML::Node topics_config_;
    rclcpp::TimerBase::SharedPtr trigger_buffer_timer_;
    std::pair<int64_t, int64_t> crop_points_{-1, -1};
};

} // namespace ros2bag_triggered

#endif // TRIGGERED_RECORDER_NODE_HPP