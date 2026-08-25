#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <linux/can.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_ipc_common/shared_memory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace relay_can_interface
{

class RelayCANSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RelayCANSystem);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_can_socket();
  void close_can_socket();
  void can_rx_loop();
  bool send_relay_frame(uint8_t mask);

  rclcpp::Logger logger_{rclcpp::get_logger("RelayCANSystem")};

  std::string can_port_{"can0"};
  uint32_t tx_can_id_{0x500};
  uint32_t rx_can_id_{0x501};
  static constexpr uint8_t PROTOCOL_VERSION = 0x01;

  int can_sock_{-1};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;

  uint8_t sequence_{0};
  std::atomic<uint8_t> desired_relay_mask_{0};
  std::atomic<uint8_t> feedback_relay_mask_{0xFF};
  std::atomic<bool> ack_received_{false};

  std::chrono::steady_clock::time_point last_send_time_{std::chrono::steady_clock::time_point::min()};
  static constexpr auto RETRY_PERIOD = std::chrono::milliseconds(250);
  uint8_t retry_count_{0};
  static constexpr uint8_t MAX_RETRIES = 2;

  double cmd_relay_mask_{0.0};
  double state_relay_mask_{0.0};

  hardware_ipc_common::SharedMemoryInterface shmitf_;
};

} // namespace relay_can_interface
