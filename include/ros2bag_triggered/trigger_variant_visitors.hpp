#ifndef ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP
#define ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP

#include <variant>
#include <std_msgs/msg/empty.hpp>

namespace ros2bag_triggered {
// Visitors
auto resetTrigger = [](auto& trigger) { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        trigger.reset();
    }
};

auto isTriggerEnabled = [](auto& trigger) -> bool { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.isEnabled();
    }
    return false; // Default behavior for monostate
};

auto getTriggerPulses = [](auto& trigger) -> std::vector<TriggerPulse> { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.getTriggerPulses();
    }
    return {};  // Return empty vector for std::monostate
};

auto getMsgType = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.getMsgType();
    }
    return "unknown"; // Default type for monostate
};

auto getName = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.getName();
    }
    return "unknown"; // Default name for monostate
};

auto getTriggerInfo = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.getTriggerInfo();
    }
    return "No trigger info available"; 
};

auto getTriggerStats = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.getTriggerStats();
    }
    return ""; // Return empty string for monostate
};

template <typename MsgType>
auto onSurge = [](auto& trigger, const typename MsgType::SharedPtr msg) -> bool {
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        // Check if the trigger's expected message type matches the provided message type
        using TriggerMsgType = typename std::decay_t<decltype(trigger)>::MsgType;
        if constexpr (std::is_same_v<TriggerMsgType, MsgType>) 
        {
            return trigger.onSurge(msg);
        } 
        else 
        {
            throw std::runtime_error("Mismatch between msg-type on trigger: " + trigger.getMsgType() + 
                                     " and msg-type on subscribed topic: " + typeid(MsgType).name());
        }
    }
    return false; // Default behavior for monostate
};

auto abortSurge = [](auto& trigger) -> bool { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.onSurge(nullptr); // Pass nullptr to indicate an abort signal
    }
    return false;
};


auto configureTrigger = [](auto& trigger, const YAML::Node& config) {   
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        // @ToDo: Support registering triggers of the same type, to different topics.
        if (trigger.isEnabled()) {   
            throw std::runtime_error("Trigger type: " + trigger.getName() + 
                                     " is already registered to a topic. Registering single trigger-type to multiple topics is not yet supported.");
        }
        using TriggerT = typename std::remove_reference<decltype(trigger)>::type;
        trigger.fromYaml(config);
        trigger.setEnabled(true);
    }
};

}; // namespace ros2bag_triggered

#endif // ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP