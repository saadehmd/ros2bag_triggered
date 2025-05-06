## Custom TriggerType Examples:

A `TriggerType` is a simple wrapper around a ROS 2 message type. It acts as a mechanism to evaluate whether a specific "triggering" condition was `True` or `False`, for how long, and how many times during the last-recorded bag. While it resembles a subscriber to a topic, it is not a node itself and does not directly subscribe to anything. Instead, it is managed by the `TriggeredRecorderNode` (discussed [here](../README.md)), which binds it to the callback of a configured topic. Each `TriggerType` instance maintains a record of all the triggers that occurred on its registered topic, and the `TriggeredRecorderNode` manages a collection of these triggers.

### Writing Your Custom TriggerType

To create a custom `TriggerType`, follow these steps:

1. Derive your class from `TriggerBase<YourDesiredROS2MsgType>`. For example:

    ```cpp
    class BatteryHealthTrigger : public TriggerBase<sensor_msgs::msg::BatteryState> // This wouldn't compile if the template argument is not recognized as a ROS2 msg type. So be sure that the template argument is ROS2 msg type and is available as a dependency to this class.
    ```
2. The basic constructor should follow the following pattern exactly:-

    ```cpp
    explicit BatteryHealthTrigger(double persistance_duration, 
                                  const rclcpp::Clock::SharedPtr clock, 
                                  const std::shared_ptr<rclcpp::Logger> logger, 
                                  bool use_msg_stamp)

        // Persistence Duration: 
        //   Duration in seconds that the trigger should persist before being activated/registered. 
        //   TriggeredRecorderNode reads this from the YAML configuration.

        // Clock: 
        //   Used by the TriggeredRecorderNode to share the Node's Clock source with each trigger instance.

        // Logger: 
        //   Used by the TriggeredRecorderNode to share the Node's Logger source with each trigger instance.

        // use_msg_stamp: 
        //   Flag to indicate whether the message's header timestamps or the Clock is used to determine 
        //   the Trigger start/end times. 
        //   NOTE: The TriggerBase interface determines at compile-time whether a message actually has 
        //   a header or not. For a message type without a header, if its associated TriggerType is 
        //   configured to use_msg_stamp, it will instead use the Clock and warn the user about this.
        //   TriggeredRecorderNode reads this from the YAML configuration.
    ```

3. Override the function ``` bool isTriggered(const YourDesiredROS2MsgType::SharedPtr)``` and implement the actual triggering-logic:- 

    ```cpp

    // battery_health_trigger.hpp
    bool isTriggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) const override;

    // battery_health_trigger.cpp
    bool BatteryHealthTrigger::isTriggered(const sensor_msgs::msg::BatteryState::SharedPtr msg) const
    {
        return msg->power_supply_health != sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
    ```
4. Override the function ```std::string getName()``` to return the name of your ```TriggerType``` as a string. It's recommended that this name matches your ```TriggerType```'s class name closely but that's not necessarily required. Note!! This is the name you'll be using to configure your trigger in the [topic_config.yaml](../config/topic_config.yaml), so be sure to name it something meaningful.

    ```cpp

    std::string getName() const override
    {
        return "BatteryHealthTrigger";
    }
    ```
5. Declare any number and type of conditional params for your triggering logic as member variables in the ```private``` or ```public``` section of your ```TriggerType``` definition. These params support the triggering logic itself and can also be configured in [topic_config.yaml](../config/topic_config.yaml). The ```BatteryHealthTrigger``` we've been looking at so far doesn't have an example of these. A good use-case of these can be seen in ```ZoneTriggerWithNavSatFix```  [header](../include/examples/zone_trigger_with_navsat_fix.hpp) and [implementation](./zone_trigger_with_navsat_fix.cpp)  


6. Override the function ```void configureConditionalParams(const YAML::Node& node)```. This function is used to read extra parameters from yaml config which weren't configured in the basic constructor. These so-called "conditional" parameters could be any kind of parameters and may or may not exist depending on your trigger-logic.

    ```cpp
    void configureConditionalParams(const YAML::Node& node) override;

    // You don't have to do any YAML exception-handling, as the base-interface takes the responsibility of calling and handling the exceptions in 
    // this funcion.
    // NOTE!! The node here is relative to each trigger in the topic_config.yaml and not relative to the root node of the whole file.
    // We'll discuss the structure of topic_config.yaml in the next step.
    
    void ZoneTriggerWithNavSatFix::configureConditionalParams(const YAML::Node& node) 
    {   
        trigger_zone_.latitude_min = node["trigger_zone"]["latitude_min"].as<double>();
        trigger_zone_.latitude_max = node["trigger_zone"]["latitude_max"].as<double>();
        trigger_zone_.longitude_min = node["trigger_zone"]["longitude_min"].as<double>();
        trigger_zone_.longitude_max = node["trigger_zone"]["longitude_max"].as<double>();
        trigger_zone_.altitude_min = node["trigger_zone"]["altitude_min"].as<double>();
        trigger_zone_.altitude_max = node["trigger_zone"]["altitude_max"].as<double>();
        
    }
    ```