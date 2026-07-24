#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "odrive_can_interface/shared_memory.hpp"
#include "hardware_system_handler/events.hpp"
#include "hardware_system_handler/fsm.hpp"

namespace hardware_system_handler
{

    class HshNode : public rclcpp::Node
    {
    public:
        HshNode();
        ~HshNode() override = default;

    private:
        rclcpp::Logger logger_{rclcpp::get_logger("HardwareSystemHandler")};

        bool open_shared_memory();
        void poll_shared_memory();
        void EventHandler();
        bool SensorActive();
        bool StartHoming();
        float CommandErrorRate();
        bool CommandVerify();
        void ControlCommand();

        rclcpp::TimerBase::SharedPtr poll_timer_;
        rclcpp::TimerBase::SharedPtr watch_dog_timer_;
        rclcpp::TimerBase::SharedPtr transmit_timer_;
        odrive_can_interface::SharedMemoryInterface shmitf_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr axis_error_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_state_pub_;

        uint32_t last_sequence_id_{0};
        bool error_checked_{false};
        bool did_one_shm_probe_{false};
        bool homing_required_{true};
        bool homing_in_progress_{false};
        bool sensors_ok_sent_{false};
        bool clear_error_pending_{false};
        bool has_timestamp_{false};
        bool state_live_{false};
        std::array<bool, SHM_MAX_AXES> homing_command_sent_{};
        std::array<bool, SHM_MAX_AXES> homing_started_{};
        std::array<bool, SHM_MAX_AXES> homing_done_{};
        std::array<int64_t, SHM_MAX_AXES> homing_command_time_ns_{};
        uint64_t last_timestamp_ns_{0};
        rclcpp::Time wait_sensors_start_{0, 0, RCL_ROS_TIME};
        rclcpp::Time wait_homing_start_{0, 0, RCL_ROS_TIME};
        rclcpp::Time last_timestamp_change_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Time clear_error_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Time fault_enter_time_{0, 0, RCL_ROS_TIME};
        std::deque<rclcpp::Time> error_events_;
        bool debug_mode_{false};
        int last_fsm_state_{-1};
        bool homing_timeout_sent_{false};
        bool offline_seen_{false};
        bool axis_online_sent_{false};
        bool offline_from_fault_{false};
        bool using_sensor{false};
        uint8_t sensor_count_{0};

        float MAX_AXIS_VEL = 200.0f;
        float MAX_AXIS_POS = 50.0f * 2.0f * 3.141592f;

    };

} // namespace hardware_system_handler
