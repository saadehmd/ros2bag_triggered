#ifndef ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP
#define ROS2BAG_TRIGGERED_TRIGGER_VARIANT_VISITORS_HPP

#include <variant>

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

auto getAllTriggers = [](auto& trigger) -> std::vector<std::pair<uint64_t, uint64_t>> { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.getAllTriggers();
    }
    return {};  // Return empty vector for std::monostate
};

auto getMsgType = [](auto& trigger) -> std::string { 
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.getMsgType();
    }
    return "unknown"; // Default type for monostate
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

auto onSurge = [](auto& trigger, const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg) -> bool {   
    if constexpr (!std::is_same_v<std::decay_t<decltype(trigger)>, std::monostate>) {
        return trigger.onSurgeSerialized(serialized_msg);
    }
    return false; // Default behavior for monostate
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