#include <test/test_helpers.hpp>
#include <examples/battery_health_trigger.hpp>
#include <gtest/gtest.h>

void surgeTests(ros2bag_triggered::tests::EmptyTrigger& triggers, double persistance_duration, size_t no_of_surges, bool use_null_termination=false, bool should_trigger=false)
{
    triggers.reset();
    auto pos_msg = std_msgs::msg::Bool().set__data(true);
    auto neg_msg = std_msgs::msg::Bool().set__data(false);

    auto pos_msg_serialized = std::make_shared<rclcpp::SerializedMessage>();
    auto neg_msg_serialized = use_null_termination ? nullptr : std::make_shared<rclcpp::SerializedMessage>();

    rclcpp::Serialization<std_msgs::msg::Bool> serializer;
    serializer.serialize_message(&pos_msg, pos_msg_serialized.get());
    if(!use_null_termination)
        serializer.serialize_message(&neg_msg, neg_msg_serialized.get());
    

    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    const float buffer_duration{0.05}; // This is added so that the test loop doesn't run slighlty short of the required persistance duration.
    const auto trigger_duration = persistance_duration + buffer_duration;
    
    for (size_t i = 0; i < no_of_surges; i++)
    {
        const auto surge_start = clock->now();
        while ((clock->now() - surge_start).seconds() < trigger_duration)
        {
            EXPECT_FALSE(triggers.onSurgeSerialized(pos_msg_serialized));
        }
        const auto surge_end = clock->now();

        if (should_trigger)
        {
            EXPECT_TRUE(triggers.onSurgeSerialized(neg_msg_serialized));
            const auto last_trigger = triggers.getAllTriggers().back(); 
            EXPECT_EQ(triggers.getAllTriggers().size(), i+1);
            EXPECT_NEAR(rclcpp::Duration::from_nanoseconds(last_trigger.first).seconds(), surge_start.seconds(), 0.0001);   
            EXPECT_NEAR(surge_end.seconds(), rclcpp::Duration::from_nanoseconds(last_trigger.second).seconds(), 0.0001);
            EXPECT_NEAR(rclcpp::Duration::from_nanoseconds(last_trigger.second - last_trigger.first).seconds(), trigger_duration, 0.0001);
        }
        else
        {
            EXPECT_FALSE(triggers.onSurgeSerialized(neg_msg_serialized));
            EXPECT_EQ(triggers.getAllTriggers().size(), 0);
        }
    }

}

TEST(HeaderLessTriggerTests, test_time_source_initialization)
{
    auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("HeaderlessTimeSourceTest"));
    EXPECT_THROW(ros2bag_triggered::tests::EmptyTrigger(1, nullptr, logger, /*use_time_stamp=*/false), std::runtime_error);
    EXPECT_THROW(ros2bag_triggered::tests::EmptyTrigger(1, nullptr, logger, /*use_time_stamp=*/true), std::runtime_error);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    EXPECT_NO_THROW(ros2bag_triggered::tests::EmptyTrigger(1, clock, logger, /*use_time_stamp=*/false));
    EXPECT_NO_THROW(ros2bag_triggered::tests::EmptyTrigger(1, clock, logger, /*use_time_stamp=*/true));
}

TEST(TiggerWithHeaderTests, test_time_source_initialization)
{
    auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("HeaderTimeSourceTest"));
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    EXPECT_THROW(ros2bag_triggered::examples::BatteryHealthTrigger(1, nullptr, logger, /*use_time_stamp=*/false), std::runtime_error);
    EXPECT_NO_THROW(ros2bag_triggered::examples::BatteryHealthTrigger(1, nullptr, logger, /*use_time_stamp=*/true));
    EXPECT_NO_THROW(ros2bag_triggered::examples::BatteryHealthTrigger(1, clock, logger, /*use_time_stamp=*/false));
}

TEST(TiggerWithHeaderTests, test_trigger_stamps)
{

    auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("StampsTest"));
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    const float persistance_duration{0.25};
    const float buffer_duration{0.05}; // This is added so that the test loop doesn't run slighlty short of the required persistance duration.
    
    auto triggers_with_stamps = ros2bag_triggered::examples::BatteryHealthTrigger(persistance_duration, clock, logger, /*use_time_stamp=*/true);
    auto triggers_with_clock = ros2bag_triggered::examples::BatteryHealthTrigger(persistance_duration, clock, logger, /*use_time_stamp=*/false);

    triggers_with_stamps.setEnabled(true);
    triggers_with_clock.setEnabled(true);

    rclcpp::Serialization<sensor_msgs::msg::BatteryState> serializer;

    const auto time_offset = rclcpp::Duration::from_seconds(10.0);
    const auto surge_start = clock->now();
    std::shared_ptr<rclcpp::Time> clock_start; 
    while ((clock->now() - surge_start).seconds() < persistance_duration + buffer_duration)
    {
        sensor_msgs::msg::BatteryState pos_msg;
        pos_msg.header.stamp = clock->now() + time_offset;
        pos_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
        
        auto pos_msg_serialized = std::make_shared<rclcpp::SerializedMessage>();
        serializer.serialize_message(&pos_msg, pos_msg_serialized.get());
    
        EXPECT_FALSE(triggers_with_stamps.onSurgeSerialized(pos_msg_serialized));

        if(!clock_start)
        {
            clock_start = std::make_shared<rclcpp::Time>(clock->now());
        }
        EXPECT_FALSE(triggers_with_clock.onSurgeSerialized(pos_msg_serialized));
    }
    

    const auto surge_end = clock->now();
    sensor_msgs::msg::BatteryState neg_msg;
    neg_msg.header.stamp = surge_end + time_offset;
    neg_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    auto neg_msg_serialized = std::make_shared<rclcpp::SerializedMessage>();
    serializer.serialize_message(&neg_msg, neg_msg_serialized.get());

    EXPECT_TRUE(triggers_with_stamps.onSurgeSerialized(neg_msg_serialized));

    auto clock_end = clock->now();
    EXPECT_TRUE(triggers_with_clock.onSurgeSerialized(neg_msg_serialized));

    const auto last_trigger_with_stamps = triggers_with_stamps.getAllTriggers().back();
    EXPECT_NEAR(rclcpp::Duration::from_nanoseconds(last_trigger_with_stamps.first).seconds(), (surge_start + time_offset).seconds(), 0.0001);
    EXPECT_NEAR(rclcpp::Duration::from_nanoseconds(last_trigger_with_stamps.second).seconds(), (surge_end + time_offset).seconds(), 0.0001);

    const auto last_trigger_with_clock = triggers_with_clock.getAllTriggers().back();
    EXPECT_NEAR(rclcpp::Duration::from_nanoseconds(last_trigger_with_clock.first).seconds(), clock_start->seconds(), 0.0001);
    EXPECT_NEAR(rclcpp::Duration::from_nanoseconds(last_trigger_with_clock.second).seconds(), clock_end.seconds(), 0.0001);
}    


TEST(AllTriggerTests, on_surge_test)
{
    auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("onSurgeSerializedTest"));
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    // No persistance case.
    auto triggers_unpersisted = ros2bag_triggered::tests::EmptyTrigger(0, clock, logger, /*use_time_stamp=*/false);
    triggers_unpersisted.setEnabled(true);
    surgeTests(triggers_unpersisted, /*trigger_duration=*/ 0, /*no_of_surges=*/10, /*use_null_termination=*/false, /*should_trigger=*/true);
    surgeTests(triggers_unpersisted, /*trigger_duration=*/ 0, /*no_of_surges=*/10, /*use_null_termination=*/ true, /*should_trigger=*/true);

    // Persistance case.
    const double trigger_duration = 0.25;
    auto triggers_persisted = ros2bag_triggered::tests::EmptyTrigger(trigger_duration, clock, logger, /*use_time_stamp=*/false);
    triggers_persisted.setEnabled(true);

    // Test false triggers with the smaller duration.
    const auto test_durations = std::vector<double>{0.05, 0.15, 0.25};
    for (const auto& duration : test_durations)
    {
        surgeTests(triggers_persisted, /*trigger_duration=*/ duration, /*no_of_surges=*/3, /*use_null_termination=*/false, /*should_trigger=*/duration == trigger_duration);
        surgeTests(triggers_persisted, /*trigger_duration=*/ duration, /*no_of_surges=*/3, /*use_null_termination=*/true, /*should_trigger=*/duration == trigger_duration);

    }
    
}

TEST(AllTriggerTests, test_enabled)
{
    auto logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("EnabledTest"));
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto triggers = ros2bag_triggered::tests::EmptyTrigger(0.1, clock, logger, /*use_time_stamp=*/true);
    surgeTests(triggers, /*trigger_duration=*/ 0.1, /*no_of_surges=*/3, /*use_null_termination=*/false, /*should_trigger=*/false);
    triggers.setEnabled(true);
    surgeTests(triggers, /*trigger_duration=*/ 0.1, /*no_of_surges=*/3, /*use_null_termination=*/false, /*should_trigger=*/true);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}