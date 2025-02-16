#include <ros2bag_triggered/triggered_writer.hpp>
#include <gtest/gtest.h>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>

using namespace ros2bag_triggered;
using namespace std::chrono_literals;

class TriggeredWriterTestHelper : public TriggeredWriter
{

  public:
    TriggeredWriterTestHelper(const rclcpp::Logger& logger) : TriggeredWriter(logger)
    {}

    ~TriggeredWriterTestHelper()
    {}

    rosbag2_storage::BagMetadata getMetaData()
    {
        return metadata_;
    }

    ros2bag_triggered::TriggeredWriter::Config getConfig() const
    {
        return config_;
    }

    std::optional<rosbag2_storage::StorageOptions> getRewriteOptions() const
    {
        return rewrite_options_;
    }

    void setMetaData(const rosbag2_storage::BagMetadata& metadata)
    {
        metadata_ = metadata;
    }

    void setConfig(const ros2bag_triggered::TriggeredWriter::Config& config)
    {
        config_ = config;
    }

    void setRewriteOptions(const rosbag2_storage::StorageOptions& rewrite_options)
    {
        rewrite_options_ = std::make_optional(rewrite_options);
    }

    void setTriggered(bool is_triggered)
    {
        is_triggered_ = is_triggered;
    }
};

class TriggeredWriterTestFixture : public ::testing::Test
{
  protected:
    TriggeredWriterTestFixture() : 
        writer_{std::make_unique<ros2bag_triggered::TriggeredWriter>(rclcpp::get_logger("TriggeredWriterTests"))}
    {}

    ~TriggeredWriterTestFixture()
    {}

    virtual void SetUp() override { }

    virtual void TearDown() override
    {}

    TriggeredWriterTestHelper & get_test_helper()
    {
        auto& base_writer = writer_.get_implementation_handle();
        auto& triggered_writer = static_cast<TriggeredWriterTestHelper&>(base_writer);
        return triggered_writer;
    }

    std::string open_valid_bag()
    {
        auto& test_helper = get_test_helper();
        rosbag2_storage::StorageOptions storage_options; 
        const rosbag2_cpp::ConverterOptions converter_options;
        std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
        test_helper.initialize(prefix_path / "test/configs/valid_config/writer_config.yaml");
        std::string bag_name = "test_bag_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        storage_options.uri = bag_name;
        //test_helper.open(storage_options, converter_options);
        EXPECT_NO_THROW(test_helper.open(storage_options, converter_options));
        EXPECT_TRUE(test_helper.is_open());
        return bag_name;
    }

    void test_close_with_triggers(bool do_crop)
    {
        auto& test_helper = get_test_helper();
        rosbag2_storage::StorageOptions storage_options; 
        const rosbag2_cpp::ConverterOptions converter_options;
        std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
        test_helper.initialize(prefix_path / "test/configs/valid_config/writer_config.yaml");

        // Set some storage options to be validated later.
        std::string bag_name = "test_bag_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        storage_options.uri = bag_name;
        const auto expected_duration_ns = 3 * 1e9;
        storage_options.storage_id = rosbag2_storage::get_default_storage_id();
        storage_options.start_time_ns = std::chrono::system_clock::now().time_since_epoch().count();
        storage_options.end_time_ns = storage_options.start_time_ns + expected_duration_ns;

        // Simulate opening bag and writing some messages within the configured storage duration.
        writer_.open(storage_options, converter_options);
        writer_.write<std_msgs::msg::Bool>(std_msgs::msg::Bool(), "test_topic", rclcpp::Time(storage_options.start_time_ns));
        writer_.write<std_msgs::msg::Bool>(std_msgs::msg::Bool(), "test_topic", rclcpp::Time(storage_options.start_time_ns + expected_duration_ns/2));
        writer_.write<std_msgs::msg::Bool>(std_msgs::msg::Bool(), "test_topic", rclcpp::Time(storage_options.start_time_ns + expected_duration_ns));

        // Simulate the conditions that might cause bag cropping and moving.
        // If the cropping is disabled, the rewrite options should have no effect on bag duration even if we set them. 
        test_helper.setTriggered(true);
        auto rewrite_options = storage_options;
        rewrite_options.start_time_ns = storage_options.start_time_ns + expected_duration_ns/4;
        test_helper.setRewriteOptions(rewrite_options);

        // Set the bag cropping option
        auto config = test_helper.get_config();
        config.bag_cropping = do_crop;
        test_helper.setConfig(config);

        const auto last_bag_name = test_helper.get_bag_name();
        writer_.close();

        // Validate the bag file exists in the right destination.
        EXPECT_TRUE(test_helper.get_bag_name().empty());
        EXPECT_FALSE(std::filesystem::exists(config.bag_root_dir + last_bag_name));
        EXPECT_TRUE(std::filesystem::exists(config.bag_root_dir + "/triggered_bags/" + last_bag_name));
        
        // Open the stored bag file and validate its metadata.
        rosbag2_storage::StorageOptions read_options;
        read_options.uri = config.bag_root_dir + "/triggered_bags/" + last_bag_name;
        read_options.storage_id = rosbag2_storage::get_default_storage_id();
        rosbag2_cpp::Reader reader;
        reader.open(read_options);

        const auto bag_meta = reader.get_metadata();
        const auto recorded_topics = bag_meta.topics_with_message_count; 

        // If cropping was enabled we should see a bag with half the duration and one less message.
        if (do_crop) {
            EXPECT_NEAR(bag_meta.duration.count(), expected_duration_ns/2, 1000);
            EXPECT_EQ(bag_meta.message_count, 2);
        } else {
            EXPECT_NEAR(bag_meta.duration.count(), expected_duration_ns, 1000);
            EXPECT_EQ(bag_meta.message_count, 3);
        }
        EXPECT_EQ(recorded_topics.size(), 1);
        EXPECT_EQ(recorded_topics[0].topic_metadata.name, "test_topic");
        EXPECT_EQ(recorded_topics[0].message_count, do_crop ? 2 : 3);
        EXPECT_NEAR(bag_meta.starting_time.time_since_epoch().count(), do_crop ? storage_options.start_time_ns + expected_duration_ns/2 : storage_options.start_time_ns, 1000);
    }
    
    rosbag2_cpp::Writer writer_;
};

TEST_F(TriggeredWriterTestFixture, test_valid_initialization)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    auto& test_helper = get_test_helper();
    EXPECT_NO_THROW(test_helper.initialize(prefix_path / "test/configs/valid_config/writer_config.yaml"));
    EXPECT_EQ(test_helper.get_config().bag_root_dir, "/test_bags");
    EXPECT_EQ(test_helper.get_config().trigger_buffer_duration, 600.0);
    EXPECT_EQ(test_helper.get_config().crop_gap, 1.0);
    EXPECT_TRUE(test_helper.get_config().bag_cropping);
    EXPECT_TRUE(test_helper.get_config().write_trigger_stats);
    EXPECT_EQ(test_helper.get_storage_options().max_bagfile_size, 1000000000);
    EXPECT_EQ(test_helper.get_storage_options().max_bagfile_duration, 300);
    EXPECT_EQ(test_helper.get_storage_options().max_cache_size, 0);
}

TEST_F(TriggeredWriterTestFixture, test_invalid_config_filenames)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    auto& test_helper = get_test_helper();
    EXPECT_THROW(test_helper.initialize(prefix_path / "test/configs/invalid_filenames_config/writer_config.yaml"), YAML::BadFile);
}

TEST_F(TriggeredWriterTestFixture, test_incomplete_config)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    auto& test_helper = get_test_helper();
    EXPECT_NO_THROW(test_helper.initialize(prefix_path / "test/configs/incomplete_config/writer_config.yaml"));
}

TEST_F(TriggeredWriterTestFixture, test_config_with_invalid_field_types)
{
    std::filesystem::path prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
    auto& test_helper = get_test_helper();
    EXPECT_THROW(test_helper.initialize(prefix_path / "test/configs/config_with_wrong_field_types/writer_config.yaml"), YAML::BadConversion);
}

TEST_F(TriggeredWriterTestFixture, test_open)
{
    auto& test_helper = get_test_helper();
    auto bag_name = open_valid_bag();
    EXPECT_TRUE(test_helper.get_bag_name() == bag_name);
    EXPECT_TRUE(test_helper.get_base_folder() == test_helper.get_config().bag_root_dir + "/" + bag_name);
    EXPECT_TRUE(std::filesystem::exists(test_helper.get_base_folder()));
}

TEST_F(TriggeredWriterTestFixture, test_close_no_triggers)
{
    auto& test_helper = get_test_helper();
    auto bag_name = open_valid_bag();
    const auto bag_base_folder = test_helper.get_base_folder();
    test_helper.close();
    EXPECT_TRUE(test_helper.get_bag_name().empty());
    EXPECT_FALSE(std::filesystem::exists(bag_base_folder));
    EXPECT_FALSE(std::filesystem::exists(test_helper.getConfig().bag_root_dir + "/triggered_bags/" + bag_name));
}


TEST_F(TriggeredWriterTestFixture, test_close_with_triggers_and_no_crop)
{
    const bool do_crop = false;
    test_close_with_triggers(do_crop);
}

TEST_F(TriggeredWriterTestFixture, test_close_with_triggers_and_crop)
{
    const bool do_crop = true;
    test_close_with_triggers(do_crop);
}

TEST_F(TriggeredWriterTestFixture, test_cropping)
{
    auto& test_helper = get_test_helper();
    auto bag_name = open_valid_bag();
    auto config = test_helper.get_config();

    const int64_t trigger_start_time = 2e9;
    const int64_t trigger_end_time = 7e9;
    const double crop_gap = test_helper.getConfig().crop_gap * 1e9;

    // Write some messages to the bag to create artifical bag duration.
    for(size_t i = 0; i <= 10; i++)
    {
        const auto stamp = rclcpp::Time(i, 0);
        writer_.write<std_msgs::msg::Bool>(std_msgs::msg::Bool(), "test_topic", stamp);
    }
    
    test_helper.set_crop_points(-1, trigger_end_time);
    auto rewrite_options = test_helper.getRewriteOptions();
    ASSERT_FALSE(rewrite_options.has_value());

    test_helper.set_crop_points(trigger_start_time, -1);
    rewrite_options = test_helper.getRewriteOptions();
    ASSERT_FALSE(rewrite_options.has_value());

    test_helper.set_crop_points(-1, -1);
    rewrite_options = test_helper.getRewriteOptions();
    ASSERT_FALSE(rewrite_options.has_value());

    config.bag_cropping = false;
    test_helper.setConfig(config);
    test_helper.set_crop_points(trigger_start_time, trigger_end_time);
    rewrite_options = test_helper.getRewriteOptions();
    ASSERT_FALSE(rewrite_options.has_value());

    config.bag_cropping = true;
    test_helper.setConfig(config);
    test_helper.set_crop_points(trigger_start_time, trigger_end_time);
    rewrite_options = test_helper.getRewriteOptions();
    ASSERT_TRUE(rewrite_options.has_value());
    EXPECT_EQ(rewrite_options.value().start_time_ns, static_cast<double>(trigger_start_time) - crop_gap);
    EXPECT_EQ(rewrite_options.value().end_time_ns, static_cast<double>(trigger_end_time) + crop_gap);
}