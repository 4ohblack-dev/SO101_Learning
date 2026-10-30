#pragma once

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace so100_control{
    class JoyTeleop : public rclcpp::Node
    {
    public:
        JoyTeleop();
    private:
        using Joy = sensor_msgs::msg::Joy;
        using JointState = sensor_msgs::msg::JointState;
        using Clock = std::chrono::steady_clock;
        static constexpr std::size_t kJointCount = 6;

        void onJoy(const Joy::SharedPtr message);
        void onState(const JointState::SharedPtr message);
        void onTimer();

        double applyDeadzone(double value) const;
        void publishTargets();

        const std::array<std::string, kJointCount> joint_names_{
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper"
        };
        const std::array<double, kJointCount> lower_limits_{
          -2.0, 0.0, -3.14158, -2.5, -3.14158, -0.2
        };
        const std::array<double, kJointCount> upper_limits_{
          2.0, 3.5, 0.0, 1.2, 3.14158, 2.0
        };
        const std::array<std::size_t, kJointCount> axis_indices_{
          0, 1, 3, 2, 6, 7
        };
        std::array<double, kJointCount> current_positions_{};
        std::array<double, kJointCount> targets_{};
        std::array<double, kJointCount> inputs_{};
        std::vector<double> speeds_;
        std::vector<double> directions_;

        double deadzone_;
        double joy_timeout_sec_;
        double state_timeout_sec_;
        double max_target_lead_rad_;

        bool have_state_{false};
        bool have_joy_{false};
        bool enable_pressed_{false};
        bool armed_{false};
        bool active_{false};

        Clock::time_point last_joy_time_{};
        Clock::time_point last_state_time_{};
        Clock::time_point last_tick_time_{};

        rclcpp::Subscription<Joy>::SharedPtr joy_subscription_;
        rclcpp::Subscription<JointState>::SharedPtr state_subscription_;
        rclcpp::Publisher<JointState>::SharedPtr command_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
