#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <deque>
#include <limits>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "hardware_ipc_common/shared_memory.hpp"
#include "hardware_system_handler/events.hpp"
#include "hardware_system_handler/fsm.hpp"

namespace hardware_system_handler
{

    class HshNode : public rclcpp::Node
    {
    public:
        explicit HshNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~HshNode() override;

    private:
        rclcpp::Logger logger_{rclcpp::get_logger("HardwareSystemHandler")};

        bool open_shared_memory();
        void poll_shared_memory();
        void EventHandler();
        bool SensorActive();
        bool refresh_state();
        bool refresh_cmd_if();
        bool refresh_sensor_state();
        void request_axis_state(size_t axis, AxisDesiredState state);
        void request_clear_errors(size_t axis);
        bool StartHoming();
        float CommandErrorRate();
        bool CommandVerify();
        void ControlCommand();

        rclcpp::TimerBase::SharedPtr poll_timer_;
        rclcpp::TimerBase::SharedPtr watch_dog_timer_;
        rclcpp::TimerBase::SharedPtr transmit_timer_;
        hardware_ipc_common::SharedMemoryInterface shmitf_;
        HwiStateBlock last_state_{};
        HwiCommandIfBlock last_cmd_if_{};
        HwiSensorStateBlock last_sensor_state_{};
        HshControlBlock desired_control_{};
        bool have_state_snapshot_{false};
        bool have_cmd_snapshot_{false};
        bool have_sensor_snapshot_{false};
        rclcpp::Time last_state_snapshot_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Time last_cmd_snapshot_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Time last_sensor_snapshot_time_{0, 0, RCL_ROS_TIME};

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr axis_error_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_state_pub_;

        uint32_t last_sequence_id_{0};
        bool error_checked_{false};
        bool did_one_shm_probe_{false};
        bool homing_required_{true};
        bool bypass_position_homing_{false};
        bool homing_in_progress_{false};
        bool sensors_ok_sent_{false};
        bool clear_error_pending_{false};
        bool has_timestamp_{false};
        bool state_live_{false};
        std::array<bool, SHM_MAX_AXES> homing_command_sent_{};
        std::array<bool, SHM_MAX_AXES> homing_started_{};
        std::array<bool, SHM_MAX_AXES> homing_done_{};
        std::array<bool, SHM_MAX_AXES> clear_error_requested_{};
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
        // Temporary bench setting: no mechanical position limit is installed.
        // Keep the field explicit so a finite bound can be restored before
        // attaching the mechanism.
        float MAX_AXIS_POS = std::numeric_limits<float>::infinity();

        bool estop_brake_latched_{false};
        float estop_brake_target_[SHM_MAX_AXES]{0.0f};
        bool brake_reference_valid_[SHM_MAX_AXES]{false};
        float brake_reference_turns_[SHM_MAX_AXES]{0.0f};

        // CAN1 Vehicle Telemetry and Mode Interlock
        std::atomic<bool> can1_running_{true};
        std::atomic<float> vehicle_speed_kmh_{0.0f};
        std::atomic<int64_t> vehicle_speed_last_ns_{0};
        std::atomic<float> vehicle_battery_v_{0.0f};
        std::thread can1_thread_;
        void can1_rx_worker();

        float stationary_speed_threshold_kmh_{0.5f};
        float estop_recovery_steer_deviation_{10.0f};
        bool effective_auto_mode_{false};
        bool effective_mode_initialized_{false};
    };

} // namespace hardware_system_handler
