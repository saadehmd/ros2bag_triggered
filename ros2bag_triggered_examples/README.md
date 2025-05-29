## Custom TriggerType Examples:

A `TriggerType` is a simple wrapper around a ROS 2 message type. It acts as a mechanism to evaluate whether a specific "triggering" condition was `True` or `False`, for how long, and how many times during the last-recorded bag. While it resembles a subscriber to a topic, it is not a node itself and does not directly subscribe to anything. Instead, it is managed by the `TriggeredRecorderNode` (discussed [here](../README.md)), which binds it to the callback of a configured topic. Each `TriggerType` instance maintains a record of all the triggers that occurred on its registered topic, and the `TriggeredRecorderNode` manages a collection of these triggers.

**Some important properties of triggers:**  

1. Triggers can be configured to be presistant i.e.; On subscribing to a msg with positive triggering condition, trigger doesn't immediately activate and waits for subsequent msgs to arrive. With every new subscription (having a positive trigger condition), it checks the amount of time passed since the beginning of last surge. If it stays positive equal-to or more than the configured persistance-duration, that trigger-pulse is registered otherwise discarded. This behavior can be by-passed by setting persistance duration to zero and trigger then activates on the first positive msg.

2. Triggers latch once activated (after persistance duration). A trigger would not be de-activated until a negative msg arrives, or until the recorder switches over to the new bag. Even though triggers stay latched for the durations where no msgs are arriving, their registered ```start_time``` and ```end_time``` are only based on recieved msgs.

3. Triggers can be configured to use the stamps from their own header or the node clock. If a triggered msg type has no header, it defaults to using the Node clock.

4. A pulse on a trigger is essentially a combination of postive and negative edges. A trigger can have one pulse for the whole duration of bag or many different pulses depending on how many times it activated and deactivated. In case the bag-switch-over occurs before the negative-edge of a recent pulse on a trigger, this pulse's end-time is the time of last received msg.

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
    // NOTE!! Each YAML node here is relative to the respective trigger in the topic_config.yaml and not relative to the root node of the whole file.
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

### Configuring the Triggers:-

The triggers are configured in the file ```topic_config.yaml```. This file is a list of topics where each topic has it's own configuration some of which is optional while some is required.

1. Each topic requires the field ```msg_type``` which cannot be left-out.
2. Under each topic an optional field ```record``` specifies whether the topic should be recorded in the bag or not. If not specified, it defaults to true. 
3. The field ```triggers``` can be left-out if a topic is only recorded and not used for any triggers. If both ```record``` field is set to ```False``` and no ```triggers``` field is present, the topic will be ignored entirely and not subscribed to.
4. Under each  ```triggers``` field, you have to configure a list of triggers.
5. Each configured trigger under ```triggers``` requires the field ```type``` which cannot be left-out. This field should match exactly the name returned by the ```getName()``` function for the ```TriggerType``` implemented in the **Writing Your Custom TriggerType** section. 
6. Each configured trigger under ```triggers``` has an optional field ```persistance_duration```, used to set it's persistance-duration in seconds. If left-out it defaults to zero i.e.; a trigger that doesn't need persistance and can activate on the very first triggering msg.
7. Each configured trigger under ```triggers``` has an optional field ```use_msg_stamp```, used to indicate whether this trigger uses stamp from it's own header or from node clock. If left-out or if the msg type is header-less, this defaults to ```False```.
8. The topic's ```msg_type``` and trigger's underlying ```<msg_type>``` should match at run-time, otherwise you'll get a runtime error. 
9. All the trigger_types configured here should be part of the ```TriggerVariant``` provided to ```TriggeredRecorderNode``` as template argument. This is discussed more in [Useage](../README.md) section on the main readme.
10. The triggers can have additional fields which are used as the so-called ```conditional_params``` described in previous section.

    ```yaml

    # Example of topics triggered but not recorded 
    /robot/battery_state:
    msg_type: sensor_msgs/msg/BatteryState
    record: False                   # Defaults to True if not provided
    triggers:
        - type: BatteryHealthTrigger
        persistance_duration: 0.1   # Defaults to 0 if not provided
        use_msg_stamp: true         # Defaults to 0 if not provided

    # Example of multiple triggers on one topic
    /robot/fix:
    msg_type: sensor_msgs/msg/NavSatFix
    record: true
    triggers:
        - type: ZoneTriggerWithNavSatFix
        use_msg_stamp: true
        persistance_duration: 0.5
        # 'trigger_zone' is an example of conditional_params
        trigger_zone: {"latitude_max": 34.55, "latitude_min": 34.15, "longitude_max": 122.55, "longitude_min": 122.15, "altitude_max": 100.0, "altitude_min": 10.0}
        - type: NavSatInvalidFixTrigger
        persistance_duration: 0.1
        use_msg_stamp: false

    # Example of topics recorded to but not triggered 
    /robot/odometry:
    record: True
    msg_type: nav_msgs/msg/Odometry
    ```