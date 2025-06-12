#ifndef TRIGGERED_RECORDER_NODE_HPP
#define TRIGGERED_RECORDER_NODE_HPP

#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <unordered_map>

#include "ros2bag_triggered/trigger_base.hpp"
#include "ros2bag_triggered/trigger_variant_visitors.hpp"
#include "ros2bag_triggered/triggered_recorder_node.hpp"
#include "ros2bag_triggered/triggered_writer.hpp"

namespace ros2bag_triggered
{

template <typename TriggerVariant>
class TriggeredRecorderNode : public rclcpp::Node
{
  static_assert(
    std::is_same_v<std::variant_alternative_t<0, TriggerVariant>, std::monostate>,
    "The first type held by TriggerVariant must be std::monostate");

public:
  TriggeredRecorderNode(
    std::string && node_name = "triggered_recorder_node",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions(),
    const std::filesystem::path & config_dir = "")
  : rclcpp::Node(std::move(node_name), node_options)
  {
    initialize(config_dir);
  }

  ~TriggeredRecorderNode() { reset(); }

protected:
  void initialize(const std::filesystem::path & config_dir)
  {
    std::filesystem::path prefix_path = config_dir;
    if (config_dir.empty())
    {
      RCLCPP_WARN(
        get_logger(), "Config directory path not provided. Using default config directory.");
      prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered") + "/config";
    }

    else if (!std::filesystem::exists(config_dir) || !std::filesystem::is_directory(config_dir))
    {
      RCLCPP_ERROR(get_logger(), "Invalid config directory path: %s", config_dir.c_str());
      throw std::runtime_error("Invalid config directory path.");
    }

    RCLCPP_INFO(
      get_logger(), "Initializing Triggered Recorder Node from config directory: %s",
      prefix_path.c_str());

    auto topic_config = prefix_path / "topic_config.yaml";
    auto writer_config = prefix_path / "writer_config.yaml";
    try
    {
      topics_config_ = YAML::LoadFile(topic_config);
      get_writer_impl().initialize(writer_config);
    } catch (YAML::BadFile & exc)
    {
      RCLCPP_ERROR(
        get_logger(), "Invalid config file path exception in topic/writer intialization. %s",
        exc.what());
      throw;
    } catch (const YAML::BadConversion & e)
    {
      RCLCPP_ERROR(
        get_logger(), "Bad-conversion in topic/writer intialization config: %s", e.what());
      throw;
    } catch (const YAML::InvalidNode & e)
    {
      RCLCPP_ERROR(get_logger(), "Invalid-node in topic/writer intialization config: %s", e.what());
      throw;
    }

    initialize_triggers(std::make_index_sequence<std::variant_size<TriggerVariant>::value>{});
    reset_writer();

    // This is an expensive operation, so we do it only once in the beginning.
    create_subscriptions();

    trigger_buffer_timer_ = create_wall_timer(
      std::chrono::duration<double>(get_writer_impl().get_config().trigger_buffer_duration),
      [this]() {
        reset();
        //TODO: Test whether the 'create_topic()' calls are necessary after the bag is closed and reopened with new storage options.
        reset_writer();
      });
  }

  void reset_writer()
  {
    RCLCPP_INFO(get_logger(), "Resetting the writer...");
    rosbag2_storage::StorageOptions storage_options;
    auto stamp = get_clock()->now().nanoseconds();
    writer_.open(std::to_string(stamp));

    //Create all topics immediately after creating the new writer
    for (const auto & topic : topics_config_)
    {
      try
      {
        auto topic_name = topic.first.as<std::string>();
        auto topic_cfg = topic.second;
        auto msg_type = topic_cfg["msg_type"].as<std::string>();
        auto is_recorded = topic_cfg["record"] ? topic_cfg["record"].as<bool>() : true;
        if (!is_recorded)
        {
          continue;
        }
        std::string serialization_format = "cdr";
        auto topic_meta = rosbag2_storage::TopicMetadata();
        topic_meta.name = topic_name;
        topic_meta.type = msg_type;
        topic_meta.serialization_format = serialization_format;
        writer_.create_topic(topic_meta);
      } catch (const YAML::BadConversion & e)
      {
        RCLCPP_ERROR(get_logger(), "Bad-conversion in parsing topic config", e.what());
        throw;
      } catch (const YAML::InvalidNode & e)
      {
        RCLCPP_ERROR(get_logger(), "Invalid-node in parsing topic config", e.what());
        throw;
      }
    }
  }

  void reset()
  {
    auto & triggered_writer = get_writer_impl();
    auto base_folder = triggered_writer.get_base_folder();
    std::string trigger_stats;
    std::unordered_map<std::string, std::string> triggers_as_json;
    bool write_trigger_stats = triggered_writer.get_config().write_trigger_stats;
    TriggerPulseMap all_trigger_pulses;

    /* Send an early termination surge to all triggers to
        check if any of them are activated and can update
        crop points for the bag. Also get trigger stats for writing later. */
    for (auto & trigger : triggers_)
    {
      // Send an abort surge to all triggers.
      crop_points_from_triggers(trigger.second);

      if (write_trigger_stats)
      {
        trigger_stats += std::visit(get_trigger_stats, trigger.second);

        std::string trigger_name = std::visit(get_trigger_name, trigger.second);
        auto trigger_pulses = std::visit(get_trigger_pulses, trigger.second);
        if (!trigger_pulses.empty())
        {
          all_trigger_pulses[trigger_name] = std::visit(get_trigger_pulses, trigger.second);
        }
        triggers_as_json[trigger_name] = std::visit(jsonify, trigger.second);
      }
      std::visit(reset_trigger, trigger.second);
    }

    triggered_writer.set_crop_points(crop_points_.first, crop_points_.second);
    crop_points_ = std::make_pair(-1, -1);  //Reset crop points for the next bag.
    writer_.close();
    triggered_writer.write_trigger_stats(trigger_stats, triggers_as_json);
    triggered_writer.plot_triggers(all_trigger_pulses);
  }

  void create_subscriptions()
  {
    for (const auto & topic : topics_config_)
    {
      auto topic_name = topic.first.as<std::string>();
      auto topic_cfg = topic.second;
      auto msg_type = topic_cfg["msg_type"].as<std::string>();
      auto is_recorded = topic_cfg["record"] ? topic_cfg["record"].as<bool>() : true;

      std::string triggers_debug_info =
        "Subscribed to topic: " + topic_name + " Msg type: " + msg_type + " with triggers: [";
      std::vector<std::reference_wrapper<TriggerVariant>> triggers;

      if (topic_cfg["triggers"].IsDefined())
      {
        for (const YAML::Node & triggers_info : topic_cfg["triggers"])
        {
          auto trigger_type = triggers_info["type"].as<std::string>();

          try
          {
            auto & trigger = triggers_.at(trigger_type);
            std::visit(
              [&triggers_info](auto & arg) { configure_trigger(arg, triggers_info); }, trigger);
            triggers_debug_info += std::visit(get_trigger_info, trigger);
            auto trigger_msg_type = std::visit(get_msg_type, trigger);
            auto trigger_type = std::visit(get_trigger_name, trigger);

            if (trigger_msg_type != msg_type)
            {
              RCLCPP_ERROR(
                get_logger(),
                "Trigger type %s does not have the expected message-type %s but instead has the "
                "msg-type: %s",
                trigger_type.c_str(), trigger_msg_type.c_str(), msg_type.c_str());
              throw std::runtime_error("Wrong msg-type on the trigger: " + trigger_type);
            }
            triggers.push_back(trigger);
          } catch (const std::out_of_range & e)
          {
            RCLCPP_ERROR(get_logger(), "Trigger type %s not found.", trigger_type.c_str());
            RCLCPP_ERROR(get_logger(), "Registered Trigger types are:-");
            for (auto & [key, value] : triggers_)
            {
              RCLCPP_ERROR(get_logger(), "- %s", key.c_str());
            }
            throw std::out_of_range(
              "Configured trigger types were not found in the compiled TriggerVariant.");
          }
        }
      }

      if (!triggers.empty())
      {
        // If the topic is triggered, we create subscription with the message type deduced at compile-time
        // from TriggerType::MsgType of any trigger on that topic.
        create_trigger_subscription(topic_name, triggers, is_recorded);
      } else if (is_recorded)
      {
        // If the topic is not triggered, we create subscription as generic, that resolves message-type at runtime.
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback = std::bind(
          &TriggeredRecorderNode::untriggered_topic_callback, this, std::placeholders::_1,
          topic_name, msg_type);

        untriggered_subscriptions_.push_back(
          create_generic_subscription(topic_name, msg_type, rclcpp::QoS(10), callback));
      } else
      {
        // If the topic is neither recorded nor triggered, we skip subscription entirely.
        RCLCPP_WARN(
          get_logger(), "Topic %s is not recorded and has no triggers. Skipping subscription...",
          topic_name.c_str());

        continue;
      }

      RCLCPP_INFO(get_logger(), (triggers_debug_info + "]").c_str());
    }
  }

  template <typename MsgType>
  void triggered_topic_callback(
    const typename MsgType::SharedPtr msg, const std::string & topic_name,
    const std::vector<std::reference_wrapper<TriggerVariant>> & triggers, bool is_recorded)
  {
    RCLCPP_DEBUG(get_logger(), "Received message on triggered topic: %s", topic_name.c_str());
    rclcpp::Time time_stamp = get_clock()->now();

    if (is_recorded)
    {
      writer_.write<MsgType>(*msg, topic_name, time_stamp);
    }

    for (auto & trigger : triggers)
    {
      crop_points_from_triggers<MsgType>(trigger, msg);
    }
  }

  void untriggered_topic_callback(
    const std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string & topic_name,
    const std::string & topic_type)
  {
    RCLCPP_DEBUG(get_logger(), "Received message on untriggered topic: %s", topic_name.c_str());
    rclcpp::Time time_stamp = get_clock()->now();

    writer_.write(msg, topic_name, topic_type, time_stamp);
  }

  void crop_points_from_triggers(TriggerVariant & trigger, bool negative_edge)
  {
    if (negative_edge)
    {
      auto trigger_pulses = std::visit(get_trigger_pulses, trigger);
      if (trigger_pulses.empty())
      {
        return;
      }

      auto & last_trigger_pulse = trigger_pulses.back();
      auto start_point = static_cast<int64_t>(last_trigger_pulse.start_time_);
      auto stop_point = static_cast<int64_t>(last_trigger_pulse.end_time_);
      crop_points_.first = crop_points_.first >= 0 ? std::min(crop_points_.first, start_point)
                                                   : last_trigger_pulse.start_time_;
      crop_points_.second = std::max(crop_points_.second, stop_point);
    }
  }

  template <typename MsgType>
  void crop_points_from_triggers(TriggerVariant & trigger, const typename MsgType::SharedPtr msg)
  {
    bool negative_edge =
      std::visit([&msg](auto & arg) { return on_surge<MsgType>(arg, msg); }, trigger);

    crop_points_from_triggers(trigger, negative_edge);
  }

  void crop_points_from_triggers(TriggerVariant & trigger)
  {
    bool negative_edge = std::visit([](auto & arg) { return abort_surge(arg); }, trigger);
    crop_points_from_triggers(trigger, negative_edge);
  }

  template <std::size_t... Is>
  void initialize_triggers(std::index_sequence<Is...>)
  {
    (([&]() {
       using TriggerT = std::variant_alternative_t<Is, TriggerVariant>;
       if constexpr (!std::is_same_v<TriggerT, std::monostate>)
       {
         auto temp = TriggerT{
           /*persistance_duration=*/0, get_clock(), std::make_shared<rclcpp::Logger>(get_logger()),
           /*use_msg_stamp=*/true};
         triggers_[temp.get_name()] = std::move(temp);
       }
     }()),
     ...);

    std::string triggers_debug_info = "Recorder Node initialized with following trigger types:-";
    for (auto & [key, value] : triggers_)
    {
      const auto msg_type = std::visit(get_msg_type, value);
      triggers_debug_info += "\n\tTrigger Type: " + key + ",\tMsg Type: " + msg_type;
    }
    RCLCPP_INFO(get_logger(), triggers_debug_info.c_str());
  }

  void create_trigger_subscription(
    std::string topic_name, const std::vector<std::reference_wrapper<TriggerVariant>> & triggers,
    bool is_recorded)
  {
    if (triggers.empty())
    {
      return;
    }

    std::visit(
      [&](auto & concrete_trigger) {
        using TriggerT = std::decay_t<decltype(concrete_trigger)>;
        if constexpr (!std::is_same_v<TriggerT, std::monostate>)
        {
          using MsgType = typename TriggerT::MsgType;
          std::function<void(typename MsgType::SharedPtr)> callback = std::bind(
            &TriggeredRecorderNode::triggered_topic_callback<MsgType>, this, std::placeholders::_1,
            topic_name, triggers, is_recorded);

          triggered_subscriptions_.push_back(
            create_subscription<MsgType>(topic_name, rclcpp::QoS(10), callback));
        }
      },
      // Use the first trigger to deduce the message type. Could be any trigger tho. They all should have the same underlying message type.
      triggers.at(0).get());
  }

  ros2bag_triggered::TriggeredWriter & get_writer_impl()
  {
    auto & base_writer = writer_.get_implementation_handle();
    auto & triggered_writer = static_cast<ros2bag_triggered::TriggeredWriter &>(base_writer);
    return triggered_writer;
  }

  rosbag2_cpp::Writer writer_{
    /*writer_impl=*/std::make_unique<ros2bag_triggered::TriggeredWriter>(get_logger())};
  std::vector<rclcpp::GenericSubscription::SharedPtr> untriggered_subscriptions_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> triggered_subscriptions_;
  std::unordered_map<std::string, TriggerVariant> triggers_;
  YAML::Node topics_config_;
  rclcpp::TimerBase::SharedPtr trigger_buffer_timer_;
  std::pair<int64_t, int64_t> crop_points_{-1, -1};
};

}  // namespace ros2bag_triggered

#endif  // TRIGGERED_RECORDER_NODE_HPP