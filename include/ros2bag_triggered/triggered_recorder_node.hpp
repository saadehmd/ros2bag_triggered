#ifndef TRIGGERED_RECORDER_NODE_HPP
#define TRIGGERED_RECORDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <ros2bag_triggered/triggered_writer.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rosbag2_storage/default_storage_id.hpp>
#include <ros2bag_triggered/triggered_recorder_node.hpp>
#include <memory>
#include <chrono>
#include <variant>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>


namespace ros2bag_triggered
{

//Visitors.
auto reset_trigger = [](auto& arg) {arg.reset();};
auto initializer = [](auto& arg, const YAML::Node& config){arg = decltype(arg)(config);};
auto isUsingMsgStamps = [](auto& arg) -> bool {return arg.isUsingMsgStamps();};
auto setClock = [](auto& arg, const rclcpp::Clock::SharedPtr& clock){arg.setClock(clock);};
auto getAllTriggers = [](auto& arg) -> std::vector<std::pair<uint64_t, uint64_t>> {return arg.getAllTriggers();};
auto onSurge = [](auto& arg, const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg) -> bool 
{   
    return arg.onSurgeSerialized(serialized_msg);
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

    void initialize(const std::optional<Config>& config)
    {
        initialize_config(config);
        trigger_buffer_timer_ = create_wall_timer(std::chrono::duration<double>(config_.trigger_buffer_duration), [this](){crop_and_reset();});
        YAML::Node topics_config;
        try
        {
            topics_config = YAML::LoadFile(config_.topic_config_path+"/topic_config.yaml");
        }
        catch(YAML::BadFile& exc)
        {
            RCLCPP_ERROR(get_logger(), "The topic-config path on param-file is invalid");
            throw exc;
        }

        initialize_triggers(std::make_index_sequence<std::variant_size<TriggerVariant>::value>{});
        reset_writer();

        // This is an expensive operation, so we do it only once in the beginning.
        create_subscriptions();

    }

    void initialize_config(const std::optional<Config>& config)
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
        config_.crop_gap = declare_parameter("crop_gap", 0.0);
    }   

    void reset_writer()
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
            std::visit(reset_trigger, trigger.second);
        }

        if(crop_points_.first < 0 || crop_points_.second < 0)
        {
            RCLCPP_WARN(get_logger(), "No triggers were detected on the bag: %s", last_bag_options_.uri.c_str());
        }

        /* Change the start and end times of the bag before closing, to impliciltly perform cropping of the bag. 
        Only supported with ROS2 >= Jazzy  */
        auto& base_writer = writer_.get_implementation_handle();
        auto& triggered_writer = static_cast<ros2bag_triggered::TriggeredWriter&>(base_writer);

        /** If there were no triggers, one or both crop points are negative and hence the writer discards all 
         *  the recorded messages during writer.close() call. */
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

            if(topic_cfg["triggers"].IsDefined())
            {   
                for (const YAML::Node& trigger_info : topic_cfg["triggers"])
                {
                    auto trigger_type = trigger_info["type"].as<std::string>();
                    auto trigger = triggers_[trigger_type];

                    std::visit([&trigger_info](auto& arg){initializer(arg, trigger_info);}, trigger);
                    if(std::visit(isUsingMsgStamps, trigger))
                    {
                            std::visit([&](auto& arg){setClock(arg, get_clock());}, trigger);
                    }        
                    std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback = 
                    std::bind(&TriggeredRecorderNode::topic_callback, this, std::placeholders::_1, topic_name, msg_type, trigger);

                    subscriptions_.push_back(create_generic_subscription(topic_name, msg_type, rclcpp::QoS(10), callback));
                }
            }
                
        }

    }

    void topic_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                                            const std::string& topic_name, 
                                                            const std::string& topic_type,
                                                            TriggerVariant& trigger)
    {  
        rclcpp::Time time_stamp = this->now();
        writer_.write(*msg, topic_name, topic_type, time_stamp);
        crop_points_from_triggers(trigger, msg);
    
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
            auto start_point = static_cast<int64_t>(last_trigger.first - config_.crop_gap);
            auto stop_point =  static_cast<int64_t>(last_trigger.second + config_.crop_gap);
            crop_points_.first = crop_points_.first >= 0  ? std::min(crop_points_.first, start_point) : last_trigger.first;
            crop_points_.second = std::max(crop_points_.second, stop_point);
            
        }
    }

    template<std::size_t... Is>
    void initialize_triggers(std::index_sequence<Is...>)
    {
        // ToDo: Implement type-check to make sure user only creates a variants of TriggerBase Derived classes.
        ((triggers_[std::variant_alternative_t<Is, TriggerVariant>::name] = std::variant_alternative_t<Is, TriggerVariant>{}), ...);
    }

    rosbag2_cpp::Writer writer_{/*writer_impl=*/std::make_unique<ros2bag_triggered::TriggeredWriter>()};
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    std::unordered_map<std::string, TriggerVariant> triggers_;
    YAML::Node topics_config_;
    Config config_;
    rclcpp::TimerBase::SharedPtr trigger_buffer_timer_;
    rosbag2_storage::StorageOptions last_bag_options_;
    std::pair<int64_t, int64_t> crop_points_{-1, -1};
};

} // namespace ros2bag_triggered

#endif // TRIGGERED_RECORDER_NODE_HPP