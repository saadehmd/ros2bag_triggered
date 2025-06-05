#ifndef ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP
#define ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP

#include <variant>
#include <std_msgs/msg/empty.hpp>

namespace ros2bag_triggered {
// Visitors
auto reset_trigger = [](auto& trigger) { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        trigger.reset();
    }
};

auto is_trigger_enabled = [](auto& trigger) -> bool { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.is_enabled();
    }
    return false; // Default behavior for monostate
};

auto get_trigger_pulses = [](auto& trigger) -> std::vector<TriggerPulse> { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.get_trigger_pulses();
    }
    return {};  // Return empty vector for std::monostate
};

auto get_msg_type = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.get_msg_type();
    }
    return "unknown"; // Default type for monostate
};

auto get_trigger_name = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.get_name();
    }
    return "unknown"; // Default name for monostate
};

auto get_trigger_info = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.get_trigger_info();
    }
    return "No trigger info available"; 
};

auto get_trigger_stats = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.get_trigger_stats();
    }
    return ""; // Return empty string for monostate
};

auto jsonify = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.jsonify();
    }
    return ""; // Return empty string for monostate
};

template <typename MsgType>
auto on_surge = [](auto& trigger, const typename MsgType::SharedPtr msg) -> bool {
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        // Check if the trigger's expected message type matches the provided message type
        using TriggerMsgType = typename std::decay_t<decltype(trigger)>::MsgType;
        if constexpr (std::is_same_v<TriggerMsgType, MsgType>) 
        {
            return trigger.on_surge(msg);
        } 
        else 
        {
            throw std::runtime_error("Mismatch between msg-type on trigger: " + trigger.get_msg_type() + 
                                     " and msg-type on subscribed topic: " + typeid(MsgType).name());
        }
    }
    return false; // Default behavior for monostate
};

auto abort_surge = [](auto& trigger) -> bool { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.on_surge(nullptr); // Pass nullptr to indicate an abort signal
    }
    return false;
};


auto configure_trigger = [](auto& trigger, const YAML::Node& config) {   
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        // @ToDo: Support registering triggers of the same type, to different topics.
        if (trigger.is_enabled()) {   
            throw std::runtime_error("Trigger type: " + trigger.get_name() + 
                                     " is already registered to a topic. Registering single trigger-type to multiple topics is not yet supported.");
        }
        trigger.from_yaml(config);
        trigger.set_enabled(true);
    }
};

}; // namespace ros2bag_triggered

#endif // ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP