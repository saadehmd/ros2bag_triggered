#include <ros2bag_triggered/triggered_writer.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/bag_rewrite.hpp>
#include <chrono>
#include <iostream>
#include <fstream>

using std::chrono::duration_cast;

namespace ros2bag_triggered
{
    void TriggeredWriter::set_crop_points(int64_t start_time, int64_t end_time)
    {   
        is_triggered_ = start_time >=0 && end_time >=0;
        if (!config_.bag_cropping || !is_triggered_)
        {
            return;
        }
        auto gap = rclcpp::Duration::from_seconds(config_.crop_gap).nanoseconds();
        start_time -= gap;
        end_time += gap;
        uint64_t bag_start_time = metadata_.starting_time.time_since_epoch().count();
        uint64_t bag_end_time = bag_start_time + duration_cast<std::chrono::nanoseconds>(metadata_.duration).count();
        uint64_t new_start_time = std::max(static_cast<uint64_t>(start_time), bag_start_time);
        uint64_t new_end_time = std::min(static_cast<uint64_t>(end_time), bag_end_time);

        // Even with cropping enabled and triggers detected, there might be no need to actually crop the bag.
        if( new_start_time != bag_start_time || new_end_time != bag_end_time)
        {
            rewrite_options_ = std::make_optional<rosbag2_storage::StorageOptions>(storage_options_);
            rewrite_options_->start_time_ns = new_start_time;
            rewrite_options_->end_time_ns = new_end_time;
        }
    }

    void TriggeredWriter::initialize(const std::optional<TriggeredWriter::Config>& config)
    {
        auto prefix_path = ament_index_cpp::get_package_prefix("ros2bag_triggered");
        
        if (config.has_value())
        {
            config_ = config.value();
            return;
        }

        YAML::Node writer_cfg = YAML::LoadFile(prefix_path + "/config/config.yaml");

        storage_options_.storage_id = rosbag2_storage::get_default_storage_id();
        storage_options_.max_bagfile_size = writer_cfg["max_bagfile_size"].as<uint64_t>();
        storage_options_.max_bagfile_duration = writer_cfg["max_bagfile_duration"].as<uint64_t>();
        storage_options_.max_cache_size = writer_cfg["max_cache_size"].as<uint64_t>();
        config_.trigger_buffer_duration = writer_cfg["trigger_buffer_interval"].as<double>();
        config_.crop_gap = writer_cfg["crop_gap"].as<double>();
        config_.bag_root_dir =  writer_cfg["bag_root_dir"].as<std::string>();
        config_.bag_cropping = writer_cfg["bag_cropping"].as<bool>();
        config_.write_trigger_stats = writer_cfg["write_trigger_stats"].as<bool>();
    }

    void TriggeredWriter::close()
    {
        auto base_folder = get_base_folder();
        RCLCPP_INFO(logger_, "Closing the bag file: %s", base_folder.c_str());
        rosbag2_cpp::writers::SequentialWriter::close();
        if(!is_triggered_)
        {
            RCLCPP_WARN(logger_, "No triggers were detected. Deleting the bag: %s.", base_folder.c_str());
            std::filesystem::remove_all(base_folder);
            return;
        }
        
        auto triggered_bag_path = config_.bag_root_dir + "/triggered_bags/" + bag_name_;
        /* Crop is simply a rewrite with new start/end timestamp information.
        Only supported with ROS2 >= Jazzy */
        bool do_crop = rewrite_options_.has_value() && config_.bag_cropping;
        if (do_crop)
        {
            storage_options_.uri = base_folder;
            rewrite_options_->uri = triggered_bag_path;
            rosbag2_transport::RecordOptions record_options;
            record_options.all_topics = true;
            RCLCPP_WARN(logger_, "Cropping enabled. Rewriting triggered bag to: %s ...", rewrite_options_->uri.c_str());
            rosbag2_transport::bag_rewrite({storage_options_}, {std::make_pair(*rewrite_options_, record_options)});
            std::filesystem::remove_all(base_folder);
        }
        else
        {   
            RCLCPP_WARN(logger_, "Cropping is disabled or not needed. Moving the triggered bag to: %s ...", triggered_bag_path.c_str());
            if (!std::filesystem::exists(config_.bag_root_dir + "/triggered_bags/" ))
            {
                std::filesystem::create_directories(config_.bag_root_dir + "/triggered_bags/" );
            }
            std::filesystem::rename(base_folder, triggered_bag_path);
        }
    }

    void TriggeredWriter::open(const rosbag2_storage::StorageOptions& storage_options, const rosbag2_cpp::ConverterOptions& converter_options)
    {
        bag_name_ = storage_options.uri;
        rewrite_options_.reset();
        is_triggered_ = false;

        // This is a bit hacky, but it circumvents the constraint that rosbag2_cpp::writers::SequentialWriter::open()
        // has no overload that takes a string for the bag path. TODO: use open(std::string) in future whenever the 
        // API has this possibility.
        auto storage_options_copy = storage_options;
        storage_options_copy.uri = config_.bag_root_dir + "/" + storage_options.uri;
        RCLCPP_INFO(logger_, "Opened new bag file: %s", storage_options_copy.uri.c_str());
        rosbag2_cpp::writers::SequentialWriter::open(storage_options_copy, converter_options);
    }
    
    void TriggeredWriter::write_trigger_stats(const std::string& trigger_stats)
    {   
        if(storage_) // Write the stats only after the bag is closed;
            close();

        if (config_.write_trigger_stats && is_triggered_)
        {   
            auto triggered_bag_path = config_.bag_root_dir + "/triggered_bags/" + bag_name_;
            auto trigger_stats_file = triggered_bag_path + "/trigger_stats.txt";
            std::ofstream stats_file(trigger_stats_file);
            if (stats_file.is_open())
            {
                RCLCPP_INFO(logger_, "Writing trigger stats to file: %s", (trigger_stats_file).c_str());
                stats_file << trigger_stats;
            }
            else
            {
                RCLCPP_ERROR(logger_, "Failed to open trigger stats file: %s", (trigger_stats_file).c_str());
            }
            stats_file.close();
        }   
    }
}

