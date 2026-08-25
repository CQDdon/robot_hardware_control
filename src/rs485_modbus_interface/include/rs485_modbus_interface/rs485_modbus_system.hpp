#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_ipc_common/shared_memory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rs485_modbus_interface
{

class Rs485ModbusSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Rs485ModbusSystem);

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
  void modbus_worker_loop();
  bool open_serial();
  void close_serial();
  uint16_t calculate_crc16(const uint8_t * data, size_t length);
  bool modbus_read_ai(uint8_t slave_id, uint16_t start_reg, uint16_t count, uint16_t * out_regs);
  bool modbus_write_ao(uint8_t slave_id, uint16_t reg, uint16_t value);

  rclcpp::Logger logger_{rclcpp::get_logger("Rs485ModbusSystem")};

  // Cấu hình từ URDF
  std::string port_{"/dev/ttyUSB0"};
  int baud_rate_{9600};
  uint8_t input_slave_id_{1};
  uint8_t output_slave_id_{2};
  uint16_t ao_register_{0x0007};   // Physical AO8 -> Register 0x0007
  std::string ao_mode_{"voltage"}; // "voltage" (0-5V) hoặc "current" (0-20mA)
  double ao_raw_max_{5000.0};      // 5000.0 cho 0-5V (mV) hoặc 20000.0 cho 0-20mA (uA)
  double ai_raw_max_{5000.0};      // 5000.0 cho 0-5V (0-5000 count/mV) hoặc 4095.0 cho 12-bit ADC raw

  int serial_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread worker_thread_;

  // Dữ liệu trao đổi an toàn (Lock-free / Atomic)
  std::atomic<bool> is_auto_mode_{false};
  std::atomic<float> e_stop_voltage_{0.0f};
  std::atomic<float> steering_pot_percent_{0.0f};
  std::atomic<float> target_throttle_voltage_{0.0f};

  // State / Command buffers cho ros2_control
  double state_is_auto_{0.0};
  double state_e_stop_{0.0};
  double state_pot_percent_{0.0};
  double cmd_throttle_{0.0};

  hardware_ipc_common::SharedMemoryInterface shmitf_;
};

} // namespace rs485_modbus_interface
