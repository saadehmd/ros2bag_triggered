#ifndef TRIGGER_PLOTTER_HPP_
#define TRIGGER_PLOTTER_HPP_

#include <Python.h>
#include <matplotlibcpp.h>
#include <chrono>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>

template< typename T, size_t HexLen=6 >
std::string to_hex( T i )
{
  std::stringstream stream;
  stream << std::setfill ('0') << std::setw(HexLen) << std::hex << i;
  return stream.str();
}

namespace ros2bag_triggered::utils
{

using TimePointNs = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
using NanoSec = std::chrono::nanoseconds;


struct PulsePlot
{
    PulsePlot(std::string trigger_name, std::string trigger_color, TriggerPulse trigger_pulse)
        : trigger_name_(trigger_name), trigger_color_(trigger_color), trigger_pulse_(trigger_pulse) {}

    std::string trigger_name_;
    std::string trigger_color_;
    TriggerPulse trigger_pulse_;
};

using TriggerPulsePlots = std::vector<PulsePlot>;

inline bool trigger_comparator(const PulsePlot& a, const PulsePlot& b)
{
    return (a.trigger_pulse_.end_time_ - a.trigger_pulse_.start_time_) > (b.trigger_pulse_.end_time_ - b.trigger_pulse_.start_time_);
}

inline void sort_trigger_pulses(const TriggerPulseMap& trigger_pulse_map, TriggerPulsePlots& trigger_pulse_plots)
{
    trigger_pulse_plots.reserve(trigger_pulse_map.size());
    constexpr int max_colors{16777215}; // 24-bit RGB color represented in hex by 0xFFFFFF
    for(const auto& [trigger_name, trigger_pulses] : trigger_pulse_map)
    {
        std::string color = "#" + to_hex(trigger_pulse_map.hash_function()(trigger_name) % max_colors);
        std::cout << "Trigger Name: " << trigger_name << " Color: " << color << std::endl;
        for (const auto& pulse : trigger_pulses)
        {
            trigger_pulse_plots.emplace_back(trigger_name, color, pulse);
        }
    }
    //The pulses need to be sorted in decreasing order of their duration, so that the shorter pulses 
    // get plotted later on top of the longer ones for the ease of visualization.
    std::sort(trigger_pulse_plots.begin(), trigger_pulse_plots.end(), trigger_comparator);
}

inline void plot_triggers(const TriggerPulseMap& trigger_pulse_map,
                          const TimePointNs& bag_start_time, 
                          const TimePointNs& bag_end_time, 
                          const std::filesystem::path& trigger_plot_file)
{
    double bag_duration = std::chrono::duration_cast<std::chrono::seconds>(bag_end_time - bag_start_time).count();

    auto timezone = std::chrono::current_zone();
    auto start_datetime = timezone->to_local(bag_start_time);
    auto start_datetime_str = std::format("{:%Y-%m-%d %H:%M:%S} {}", start_datetime, timezone->name());
    auto end_datetime = timezone->to_local(TimePointNs(bag_end_time));
    auto end_datetime_str = std::format("{:%Y-%m-%d %H:%M:%S} {}", end_datetime, timezone->name());

    // Restrict the x-ticks for ease of plot-visualization.
    const size_t x_divs = 20;
    std::vector<double> xticks{};
    std::vector<double> yticks{0.0, 1.0};
    for (double x_div = 1; x_div <= x_divs; x_div++)
    {
        const double precision = 0.01;
        auto x_tick = std::round(((x_div * bag_duration) / x_divs) / precision) * precision;
        xticks.push_back(x_tick);
    }
    // @todo: the std::ranges::iota_view::to<vector>() is not available in C++20 and not standardized for all
    // compilers yet in C++23. So this could be converted to the ranges method instead of for loop in future.
    const double dt = 0.1;  // 1ms
    std::vector<double> time_axis;
    for (double t = 0; t < bag_duration; t += dt)
    {
        time_axis.push_back(t);
    }

    matplotlibcpp::figure_size(720, 480);
    try
    {
        TriggerPulsePlots trigger_pulse_plots;
        sort_trigger_pulses(trigger_pulse_map, trigger_pulse_plots);

        for (const auto& trigger_pulse_plot : trigger_pulse_plots)
        {
            std::map<std::string, std::string> plot_options;

            static std::set<std::string> plotted_triggers;
            if (plotted_triggers.find(trigger_pulse_plot.trigger_name_) == plotted_triggers.end())
            {
                // Add the trigger label to the plot options only once
                plot_options["label"] = trigger_pulse_plot.trigger_name_;
                plotted_triggers.insert(trigger_pulse_plot.trigger_name_);
            }
            plot_options["color"] = trigger_pulse_plot.trigger_color_;
            std::vector<double> trigger_axis(time_axis.size(), -1.0);

            auto& trigger_pulse = trigger_pulse_plot.trigger_pulse_;
            int start_idx = std::round((trigger_pulse.start_time_ - bag_start_time.time_since_epoch().count()) * 1e-9 / dt);
            int end_idx = std::round((trigger_pulse.end_time_ - bag_start_time.time_since_epoch().count()) * 1e-9 / dt);
            start_idx = std::clamp<int>(start_idx, 0, time_axis.size());
            end_idx = std::clamp<int>(end_idx, start_idx, time_axis.size());

            auto x_fill = std::vector<double>(time_axis.begin() + start_idx, time_axis.begin() + end_idx);
            std::vector<double> y_fill(x_fill.size(), 1.0);

            for (size_t i = start_idx; i < end_idx; ++i)
            {
                trigger_axis.at(i) = 1.0;
            }

            matplotlibcpp::plot(time_axis, trigger_axis, plot_options);
            matplotlibcpp::fill_between(x_fill, std::vector<double>(x_fill.size(), 0.95), y_fill,
                                        std::map<std::string, std::string>{{"color", trigger_pulse_plot.trigger_color_}});
            
        }

        matplotlibcpp::xlabel("Time (s)");
        matplotlibcpp::ylabel("Trigger Pulses");
        matplotlibcpp::xticks(xticks, std::map<std::string, std::string>{{"fontsize", "5"}});
        matplotlibcpp::yticks(yticks);
        matplotlibcpp::ylim(0.0, 1.0);
        matplotlibcpp::legend();
        matplotlibcpp::suptitle("Bag Start Time: " + start_datetime_str + "\nBag End Time: " + end_datetime_str,
                                {{"x", "0.5"}, {"y", "0.9"}, {"ha", "center"}, {"va", "center"}});

        matplotlibcpp::tight_layout();
        std::cout << "Saving trigger plot to: " << trigger_plot_file << std::endl;
        matplotlibcpp::save(trigger_plot_file);
    }
    catch (const std::runtime_error& e)
    {
        // first, print the generic C++ exception
        std::cerr << "[C++] matplotlibcpp::bar failed: " << e.what() << "\n";
        // next, pull the real Python exception out of the interpreter:
        if (PyErr_Occurred())
        {
            PyErr_Print();  // prints Python traceback & error message to stderr
            PyErr_Clear();
        }
        // re-throw for detailed traceback
        throw;
    }
}
}


#endif  // TRIGGER_PLOTTER_HPP_