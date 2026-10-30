#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "SMS_STS.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

using namespace std::chrono_literals;

class ServoBus : public SMS_STS
{
public:
    ~ServoBus()
    {
        if(fd >= 0){
            ::close(fd);
            fd = -1;
        }
    }
    ServoBus() = default;
    ServoBus(const ServoBus &) = delete;
};

class STS3215Node : public rclcpp::Node
{
public:
    STS3215Node() : Node("sts3215_node")
    {
        const auto port = declare_parameter<std::string>("port","/dev/ttyUSB0");
        motion_enabled_ = declare_parameter<bool>("enable_motion",false);
        ids_ = declare_parameter<std::vector<int64_t>>("servo_ids",{1,2,3,4,5,6});
        zeros_ = declare_parameter<std::vector<int64_t>>("zero_ticks", std::vector<int64_t>{});
        directions_ = declare_parameter<std::vector<int64_t>>("directions", std::vector<int64_t>{});
        min_ticks_ = declare_parameter<std::vector<int64_t>>("min_ticks",std::vector<int64_t>{});
        max_ticks_ = declare_parameter<std::vector<int64_t>>("max_ticks",std::vector<int64_t>{});

        validateSettings();
        RCLCPP_INFO(get_logger(), "Settings validated for %s. Hardware communication is not implemented yet.", port.c_str());
    }

private:
    bool motion_enabled_{false};
    std::vector<int64_t> ids_, zeros_, directions_, min_ticks_, max_ticks_;

    void validateSettings() const
    {
        constexpr std::size_t count = 6;
        if (ids_.size() != count || zeros_.size() != count ||
            directions_.size() != count || min_ticks_.size() != count ||
            max_ticks_.size() != count) {
            throw std::invalid_argument("All calibration arrays must contain 6 values");
        }
        for (std::size_t i = 0; i < count; ++i) {
            if (ids_[i] < 1 || ids_[i] > 253 ||
                std::count(ids_.begin(), ids_.end(), ids_[i]) != 1 ||
                zeros_[i] < 0 || zeros_[i] > 4095 ||
                (directions_[i] != 1 && directions_[i] != -1) ||
                min_ticks_[i] < 0 || max_ticks_[i] > 4095 ||
                min_ticks_[i] >= max_ticks_[i]) {
                throw std::invalid_argument("Invalid calibration at joint index " + std::to_string(i));
            }
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<STS3215Node>());
    } catch (const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("sts3215_node"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
