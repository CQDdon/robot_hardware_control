#include "rs485_modbus_interface/rs485_modbus_system.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "pluginlib/class_list_macros.hpp"

namespace rs485_modbus_interface
{

hardware_interface::CallbackReturn Rs485ModbusSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("port"))
    port_ = info.hardware_parameters.at("port");
  if (info.hardware_parameters.count("baud_rate"))
    baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
  if (info.hardware_parameters.count("input_slave_id"))
    input_slave_id_ = static_cast<uint8_t>(std::stoi(info.hardware_parameters.at("input_slave_id")));
  if (info.hardware_parameters.count("output_slave_id"))
    output_slave_id_ = static_cast<uint8_t>(std::stoi(info.hardware_parameters.at("output_slave_id")));
  if (info.hardware_parameters.count("ao_register"))
    ao_register_ = static_cast<uint16_t>(std::stoul(info.hardware_parameters.at("ao_register"), nullptr, 0));
  if (info.hardware_parameters.count("ao_mode"))
    ao_mode_ = info.hardware_parameters.at("ao_mode");
  if (info.hardware_parameters.count("ao_raw_max"))
    ao_raw_max_ = std::stod(info.hardware_parameters.at("ao_raw_max"));
  if (info.hardware_parameters.count("ai_raw_max"))
    ai_raw_max_ = std::stod(info.hardware_parameters.at("ai_raw_max"));

  (void)shmitf_.open(false);

  RCLCPP_INFO(logger_, "Rs485ModbusSystem initialized: port=%s, baud=%d, input_id=%d, output_id=%d, ao_reg=0x%04X, ao_mode=%s, ao_max=%.1f, ai_max=%.1f",
              port_.c_str(), baud_rate_, input_slave_id_, output_slave_id_, ao_register_, ao_mode_.c_str(), ao_raw_max_, ai_raw_max_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rs485ModbusSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rs485ModbusSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    RCLCPP_WARN(logger_, "Cannot open serial port %s (device might be offline during simulation/testing)", port_.c_str());
  }

  running_.store(true);
  worker_thread_ = std::thread(&Rs485ModbusSystem::modbus_worker_loop, this);
  RCLCPP_INFO(logger_, "Rs485ModbusSystem worker thread started");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rs485ModbusSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  running_.store(false);
  if (worker_thread_.joinable())
  {
    worker_thread_.join();
  }
  close_serial();
  RCLCPP_INFO(logger_, "Rs485ModbusSystem stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Rs485ModbusSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back("rs485", "is_auto_mode", &state_is_auto_);
  out.emplace_back("rs485", "e_stop_voltage", &state_e_stop_);
  out.emplace_back("rs485", "steering_pot_percent", &state_pot_percent_);
  return out;
}

std::vector<hardware_interface::CommandInterface> Rs485ModbusSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back("rs485", "throttle_voltage", &cmd_throttle_);
  return out;
}

hardware_interface::return_type Rs485ModbusSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const bool auto_mode = is_auto_mode_.load(std::memory_order_relaxed);
  const float e_stop = e_stop_voltage_.load(std::memory_order_relaxed);
  const float pot = steering_pot_percent_.load(std::memory_order_relaxed);

  state_is_auto_ = auto_mode ? 1.0 : 0.0;
  state_e_stop_ = static_cast<double>(e_stop);
  state_pot_percent_ = static_cast<double>(pot);

  if (shmitf_.ready())
  {
    HwiStateBlock state{};
    if (shmitf_.read_state(state))
    {
      state.rs485.is_auto_mode = auto_mode ? 1 : 0;
      state.rs485.e_stop_voltage = e_stop;
      state.rs485.steering_pot_percent = pot;
      (void)shmitf_.write_state(state);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Rs485ModbusSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  float throttle = static_cast<float>(cmd_throttle_);

  if (shmitf_.ready())
  {
    HshControlBlock ctrl{};
    if (shmitf_.read_control(ctrl))
    {
      if (!ctrl.motion_enable)
      {
        throttle = 0.0f;
      }
    }

    HwiCommandIfBlock cmd_if{};
    if (shmitf_.read_cmd_if(cmd_if))
    {
      if (cmd_if.rs485.throttle_voltage != throttle)
      {
        cmd_if.rs485.throttle_voltage = throttle;
        (void)shmitf_.write_cmd_if(cmd_if);
      }
    }
  }

  target_throttle_voltage_.store(throttle, std::memory_order_relaxed);
  return hardware_interface::return_type::OK;
}

uint16_t Rs485ModbusSystem::calculate_crc16(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i)
  {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b)
    {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

bool Rs485ModbusSystem::open_serial()
{
  if (serial_fd_ >= 0) return true;

  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) return false;

  struct termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    close_serial();
    return false;
  }

  cfsetospeed(&tty, B9600);
  cfsetispeed(&tty, B9600);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 2; // 200ms timeout

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    close_serial();
    return false;
  }
  return true;
}

void Rs485ModbusSystem::close_serial()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool Rs485ModbusSystem::modbus_read_ai(uint8_t slave_id, uint16_t start_reg, uint16_t count, uint16_t * out_regs)
{
  if (serial_fd_ < 0) return false;

  uint8_t req[8];
  req[0] = slave_id;
  req[1] = 0x04; // Read Input Registers
  req[2] = static_cast<uint8_t>((start_reg >> 8) & 0xFF);
  req[3] = static_cast<uint8_t>(start_reg & 0xFF);
  req[4] = static_cast<uint8_t>((count >> 8) & 0xFF);
  req[5] = static_cast<uint8_t>(count & 0xFF);
  uint16_t crc = calculate_crc16(req, 6);
  req[6] = static_cast<uint8_t>(crc & 0xFF);
  req[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  tcflush(serial_fd_, TCIOFLUSH);
  if (::write(serial_fd_, req, 8) != 8) return false;

  const size_t expected_len = 5 + 2 * count;
  uint8_t resp[64];
  size_t total_read = 0;

  while (total_read < expected_len)
  {
    ssize_t n = ::read(serial_fd_, resp + total_read, expected_len - total_read);
    if (n <= 0) break;
    total_read += static_cast<size_t>(n);
  }

  if (total_read != expected_len) return false;

  uint16_t resp_crc = static_cast<uint16_t>(resp[expected_len - 2] | (resp[expected_len - 1] << 8));
  if (resp_crc != calculate_crc16(resp, expected_len - 2)) return false;
  if (resp[0] != slave_id || resp[1] != 0x04 || resp[2] != count * 2) return false;

  for (uint16_t i = 0; i < count; ++i)
  {
    out_regs[i] = static_cast<uint16_t>((resp[3 + 2 * i] << 8) | resp[4 + 2 * i]);
  }
  return true;
}

bool Rs485ModbusSystem::modbus_write_ao(uint8_t slave_id, uint16_t reg, uint16_t value)
{
  if (serial_fd_ < 0) return false;

  uint8_t req[8];
  req[0] = slave_id;
  req[1] = 0x06; // Write Single Register
  req[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
  req[3] = static_cast<uint8_t>(reg & 0xFF);
  req[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
  req[5] = static_cast<uint8_t>(value & 0xFF);
  uint16_t crc = calculate_crc16(req, 6);
  req[6] = static_cast<uint8_t>(crc & 0xFF);
  req[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  tcflush(serial_fd_, TCIOFLUSH);
  if (::write(serial_fd_, req, 8) != 8) return false;

  uint8_t resp[8];
  size_t total_read = 0;
  while (total_read < 8)
  {
    ssize_t n = ::read(serial_fd_, resp + total_read, 8 - total_read);
    if (n <= 0) break;
    total_read += static_cast<size_t>(n);
  }

  if (total_read != 8) return false;
  uint16_t resp_crc = static_cast<uint16_t>(resp[6] | (resp[7] << 8));
  return (resp_crc == calculate_crc16(resp, 6) && std::memcmp(req, resp, 6) == 0);
}

void Rs485ModbusSystem::modbus_worker_loop()
{
  uint16_t ai_regs[3] = {0, 0, 0}; // 0x0005, 0x0006, 0x0007
  uint16_t last_ao_val = 0xFFFF;

  while (running_.load(std::memory_order_relaxed))
  {
    if (serial_fd_ < 0)
    {
      (void)open_serial();
      if (serial_fd_ < 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
    }

    // 1. Đọc AI 0x0005..0x0007 từ Input Module (ID = 1)
    if (modbus_read_ai(input_slave_id_, 0x0005, 3, ai_regs))
    {
      const float ai_max = (ai_raw_max_ > 0.0) ? static_cast<float>(ai_raw_max_) : 5000.0f;
      const float v_auto = std::clamp((static_cast<float>(ai_regs[0]) / ai_max) * 5.0f, 0.0f, 5.0f);
      const float v_estop = std::clamp((static_cast<float>(ai_regs[1]) / ai_max) * 5.0f, 0.0f, 5.0f);
      const float pot_pct = std::clamp((static_cast<float>(ai_regs[2]) / ai_max) * 100.0f, 0.0f, 100.0f);

      // Logic gốc: > 4.7V là Manual, <= 4.7V là Auto
      // is_auto_mode_.store(v_auto <= 4.7f, std::memory_order_relaxed);
      // Tạm thời ép luôn Auto Mode (luôn true) để test:
      is_auto_mode_.store(true, std::memory_order_relaxed);
      e_stop_voltage_.store(v_estop, std::memory_order_relaxed);
      steering_pot_percent_.store(pot_pct, std::memory_order_relaxed);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // 2. Ghi AO8 (Register ao_register_, mặc định 0x0007) xuống Output Module (ID = 2)
    const float throttle = std::clamp(target_throttle_voltage_.load(std::memory_order_relaxed), 0.0f, 5.0f);
    uint16_t ao_raw = static_cast<uint16_t>(std::clamp(std::lround((throttle / 5.0f) * ao_raw_max_), 0L, 65535L));

    if (ao_raw != last_ao_val)
    {
      if (modbus_write_ao(output_slave_id_, ao_register_, ao_raw))
      {
        last_ao_val = ao_raw;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // ~30-40Hz
  }

  (void)modbus_write_ao(output_slave_id_, ao_register_, 0);
}

} // namespace rs485_modbus_interface

PLUGINLIB_EXPORT_CLASS(rs485_modbus_interface::Rs485ModbusSystem,
                       hardware_interface::SystemInterface)
