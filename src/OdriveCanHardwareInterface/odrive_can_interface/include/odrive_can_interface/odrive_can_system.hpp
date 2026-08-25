#pragma once

#include <vector>
#include <string>
#include <memory>
#include <cstdint>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"



#include "odrive_can_interface/odrive_motor.hpp"
#include "odrive_can_interface/can_comm.hpp"
#include "hardware_ipc_common/shared_memory.hpp"

namespace odrive_can_interface
{

    class OdriveCANSystem : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(OdriveCANSystem);

        // Lifecycle
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        // Interfaces
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // I/O
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        rclcpp::Logger logger_{rclcpp::get_logger("OdriveCANSystem")};
        rclcpp::Logger can_transmit_logger_{rclcpp::get_logger("Can Transmit")};
        rclcpp::Logger can_receive_logger_{rclcpp::get_logger("Can Receive")};
        rclcpp::Logger watch_dog_logger_{rclcpp::get_logger("Watch Dog")};

        // HW params (from URDF <hardware><param>)
        std::string can_port_{"can0"};
        int baud_rate_{500000};
        bool use_velocity_command_{true};

        // Transport + driver
        std::shared_ptr<CANInterface> can_;
        std::vector<std::shared_ptr<OdriveMotor>> motors_;

        // Buffers ROS 2 Control
        std::vector<double> position_;
        std::vector<double> velocity_;
        // System-level state exported for safety controllers. Values are 0.0/1.0.
        double hardware_online_{0.0};
        double hardware_fault_{1.0};
        double motion_allowed_{0.0};

        std::vector<OdriveMotor::ControlMode> joint_mode_;
        std::vector<double> command_pos_;
        std::vector<double> command_vel_;
        // A position axis can be explicitly idled (for a spring-return brake).
        // 0.0 requests closed loop, values >= 0.5 request IDLE.
        std::vector<double> command_idle_;
        std::vector<uint64_t> last_desired_state_generation_;
        std::vector<uint64_t> last_clear_error_req_id_;
        std::vector<bool> procedural_state_started_;
        std::vector<std::chrono::steady_clock::time_point> last_axis_state_send_time_;
        std::vector<float> last_sent_target_;
        std::vector<std::chrono::steady_clock::time_point> last_target_send_time_;
        static constexpr std::chrono::milliseconds TARGET_HEARTBEAT_PERIOD{1000};

    // Shared memory 
        hardware_ipc_common::SharedMemoryInterface shmitf_;
    // Threads
        std::thread can_receive_thread_;
        std::thread watch_dog_thread_;
        std::thread can_interface_thread_;

    // Control flag
        std::atomic<bool> running_{false};
        std::atomic<bool> fatal_error_{false};
        std::atomic<bool> deactivating_{false};
        std::string last_error_;


    // Thread functions
        void CanReceive();
        void WatchDog();
        void CanInterface();
    // Frequency control
        using clock = std::chrono::steady_clock;
        static constexpr std::chrono::microseconds TX_FRE{5000}; // 200 Hz
        static constexpr std::chrono::microseconds RX_FRE{5000}; // 200 Hz
        static constexpr std::chrono::microseconds WATCH_DOG_FRE{100000}; // 10 Hz  
        static constexpr std::chrono::milliseconds AXIS_STATE_RETRY_PERIOD{100};
        static constexpr std::chrono::milliseconds IO_RETRY_PERIOD{100};
        // Diagnostic-only timeout for the MCU's 0x101 output acknowledgement.
        // It must never affect HSH motion permission or ODrive control.
        static constexpr std::chrono::milliseconds IO_ACK_TIMEOUT{500};

    };  


} // namespace odrive_can_interface
