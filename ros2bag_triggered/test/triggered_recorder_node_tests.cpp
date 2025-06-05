#include <ros2bag_triggered/triggered_recorder_node.hpp>
#include <test/test_helpers.hpp>
#include <gtest/gtest.h>

using namespace ros2bag_triggered;

using TriggerVariant = std::variant<std::monostate, tests::EmptyTrigger, tests::BatteryHealthTrigger>;

class TriggeredRecorderNodeTestHelper : public TriggeredRecorderNode<TriggerVariant>
{
public:
    TriggeredRecorderNodeTestHelper(std::string&& node_name = "triggered_recorder_node_test_helper", 
                                    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(), 
                                    const std::string& config_dir = "")
        : TriggeredRecorderNode<TriggerVariant>(std::move(node_name), node_options, config_dir)
    {
        battery_pub = create_publisher<sensor_msgs::msg::BatteryState>("robot/battery_state", 1);
        bool_pub = create_publisher<std_msgs::msg::Bool>("empty_msg", 1);
    }

    std::unordered_map<std::string, TriggerVariant> get_triggers() const
    {
        return triggers_;
    }
    size_t get_number_of_subscriptions() const
    {
        return triggered_subscriptions_.size() + untriggered_subscriptions_.size();
    }

    YAML::Node get_topics_config() const
    {
        return topics_config_;
    }

    std::filesystem::path get_bag_path() 
    {
        return get_writer_impl().get_base_folder();
    }

    std::filesystem::path get_root_dir() 
    {
        return get_writer_impl().get_config().bag_root_dir;
    }

    bool is_writer_initialized()
    {
        auto& triggered_writer = get_writer_impl();
        auto storage_opts = triggered_writer.get_storage_options();
        return triggered_writer.is_open() && !triggered_writer.get_bag_name().empty() && !storage_opts.uri.empty();
    }

    size_t get_recorded_topic_count()
    {
        auto& triggered_writer = get_writer_impl();
        return triggered_writer.get_topic_count();
    }

    bool is_trigger_buffer_timer_initialized() const
    {
        return trigger_buffer_timer_ != nullptr;
    }

    std::pair<int64_t, int64_t> get_crop_points() const
    {
        return crop_points_;
    }

    void test_init(size_t expected_topics_recorded, size_t expected_subscriptions, std::vector<std::string> expected_trigger_types)
    {
        EXPECT_EQ(get_recorded_topic_count(), expected_topics_recorded);
        EXPECT_EQ(triggers_.size(), std::variant_size<TriggerVariant>::value - 1);
        EXPECT_EQ(get_number_of_subscriptions(), expected_subscriptions);

        for (const auto& trigger_type : expected_trigger_types)
        {
            EXPECT_TRUE(triggers_.find(trigger_type) != triggers_.end());
        }

        for (const auto& trigger : triggers_)
        {
            if(std::find(expected_trigger_types.begin(), expected_trigger_types.end(), trigger.first) == expected_trigger_types.end())
            {
                EXPECT_FALSE(std::visit(is_trigger_enabled, trigger.second));
                continue;
            }
            EXPECT_TRUE(std::visit(is_trigger_enabled, trigger.second));
        }
        EXPECT_TRUE(is_writer_initialized());
        EXPECT_TRUE(trigger_buffer_timer_ != nullptr);
    }

    void call_reset() { reset(); }

    void publish_battery_msg(bool is_activated)
    {
        using sensor_msgs::msg::BatteryState;
        auto battery_status = is_activated ? BatteryState::POWER_SUPPLY_HEALTH_DEAD : BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        auto msg = sensor_msgs::msg::BatteryState().set__power_supply_health(battery_status);
        battery_pub->publish(msg);
        rclcpp::spin_some(this->get_node_base_interface());
    }

    void publish_bool_msg(bool is_activated)
    {
        auto msg = std_msgs::msg::Bool().set__data(is_activated);
        bool_pub->publish(msg);
        rclcpp::spin_some(this->get_node_base_interface());
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub;
};

class TriggeredRecorderNodeTestFixture : public ::testing::Test
{
protected:
    TriggeredRecorderNodeTestFixture()
    {}

    ~TriggeredRecorderNodeTestFixture()
    {
        rclcpp::shutdown();
        if(test_helper_)
        {
            const auto untriggered_bag_path = test_helper_->get_bag_path();
            const auto triggered_bag_path = test_helper_->get_root_dir() / "triggered_bags";
            test_helper_.reset();
            // Remove the triggered and untriggered bag directories after the test.
            if(std::filesystem::exists(triggered_bag_path))
            {
                std::filesystem::remove_all(triggered_bag_path);
            }
            if(std::filesystem::exists(untriggered_bag_path))
            {
                std::filesystem::remove_all(untriggered_bag_path);
            }
        }
    }

    virtual void SetUp() override {}

    void SetUp(const std::string& config_dir)
    {
        rclcpp::init(0, nullptr);
        test_helper_ = std::make_shared<TriggeredRecorderNodeTestHelper>("triggered_recorder_node_test_helper", rclcpp::NodeOptions(), config_dir);
    }

    virtual void TearDown() override
    {}

    std::shared_ptr<TriggeredRecorderNodeTestHelper> test_helper_;
};

TEST_F(TriggeredRecorderNodeTestFixture, test_valid_initialization)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/valid_config");

    const size_t expected_topics_recorded = 2;
    std::vector<std::string> expected_trigger_types = {"EmptyTrigger", "BatteryHealthTrigger"};
    const size_t expected_subscriptions = 3; //Topics that are neither triggered nor recorded are not added to subscriptions.

    test_helper_->test_init(expected_topics_recorded, expected_subscriptions, expected_trigger_types);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_invalid_config_dir)
{
    EXPECT_THROW(SetUp("invalid_path"), std::runtime_error);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_invalid_config_filenames)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/invalid_filenames_config"), YAML::BadFile);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_config_with_extra_triggers)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/config_with_extra_triggers"), std::out_of_range);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_config_with_invalid_trigger_names)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/config_with_invalid_trigger_names"), std::out_of_range);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_config_with_missing_triggers)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/config_with_missing_triggers");

    const size_t expected_topics_recorded = 1;
    std::vector<std::string> expected_trigger_types = {"EmptyTrigger"};
    const size_t expected_subscriptions = 2; //Topics that are neither triggered nor recorded are not added to subscriptions.

    test_helper_->test_init(expected_topics_recorded, expected_subscriptions, expected_trigger_types);    
}

TEST_F(TriggeredRecorderNodeTestFixture, test_incomplete_trigger_configs)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    EXPECT_THROW(SetUp(prefix_path / "test/configs/incomplete_trigger_config"), YAML::InvalidNode);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_topic_callback)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/valid_config");
    const auto trigger_duration = rclcpp::Duration::from_seconds(2.2);

    auto start1 = test_helper_->get_clock()->now();
    while(test_helper_->get_clock()->now() - start1 < trigger_duration)
    {
        test_helper_->publish_battery_msg(/*is_activated=*/true);
    }
    test_helper_->publish_battery_msg(/*is_activated=*/false);

    auto start2 = test_helper_->get_clock()->now();
    while(test_helper_->get_clock()->now() - start2 < trigger_duration)
    {
        test_helper_->publish_bool_msg(/*is_activated=*/true);
    }
    test_helper_->publish_bool_msg(/*is_activated=*/false);

    auto triggers = test_helper_->get_triggers();
    auto crop_points = test_helper_->get_crop_points();
    EXPECT_TRUE(std::visit(get_trigger_pulses, triggers.at("BatteryHealthTrigger")).size() == 1);
    EXPECT_TRUE(std::visit(get_trigger_pulses, triggers.at("EmptyTrigger")).size() == 1);
    EXPECT_TRUE(crop_points.first > 0 && crop_points.second > 0);
    EXPECT_EQ(crop_points.first , std::visit(get_trigger_pulses, triggers.at("BatteryHealthTrigger")).at(0).start_time_);
    EXPECT_EQ(crop_points.second , std::visit(get_trigger_pulses, triggers.at("EmptyTrigger")).at(0).end_time_);
}

TEST_F(TriggeredRecorderNodeTestFixture, test_reset)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    SetUp(prefix_path / "test/configs/valid_config");
    const auto trigger_duration = rclcpp::Duration::from_seconds(2.2);

    auto start1 = test_helper_->get_clock()->now();
    while(test_helper_->get_clock()->now() - start1 < trigger_duration)
    {
        test_helper_->publish_battery_msg(/*is_activated=*/true);
        test_helper_->publish_bool_msg(/*is_activated=*/true);
    }
    test_helper_->publish_battery_msg(/*is_activated=*/false);
    test_helper_->publish_bool_msg(/*is_activated=*/false);
    test_helper_->call_reset();
    auto triggers = test_helper_->get_triggers();
    auto crop_points = test_helper_->get_crop_points();
    EXPECT_TRUE(std::visit(get_trigger_pulses, triggers.at("BatteryHealthTrigger")).size() == 0);
    EXPECT_TRUE(std::visit(get_trigger_pulses, triggers.at("EmptyTrigger")).size() == 0);
    EXPECT_TRUE(crop_points.first == -1 && crop_points.second == -1);
    EXPECT_FALSE(test_helper_->is_writer_initialized());
}
