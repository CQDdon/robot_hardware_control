#include "relay_can_interface/relay_can_system.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "pluginlib/class_list_macros.hpp"

namespace relay_can_interface
{

hardware_interface::CallbackReturn RelayCANSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("can_port"))
    can_port_ = info.hardware_parameters.at("can_port");
  if (info.hardware_parameters.count("tx_can_id"))
    tx_can_id_ = static_cast<uint32_t>(std::stoul(info.hardware_parameters.at("tx_can_id"), nullptr, 0));
  if (info.hardware_parameters.count("rx_can_id"))
    rx_can_id_ = static_cast<uint32_t>(std::stoul(info.hardware_parameters.at("rx_can_id"), nullptr, 0));

  (void)shmitf_.open(false);

  RCLCPP_INFO(logger_, "RelayCANSystem initialized: port=%s, tx_id=0x%03X, rx_id=0x%03X",
              can_port_.c_str(), tx_can_id_, rx_can_id_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RelayCANSystem::on_configure(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RelayCANSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_can_socket())
  {
    RCLCPP_WARN(logger_, "Failed to open CAN socket on %s (CAN interface may be offline during simulation/testing)", can_port_.c_str());
  }

  running_.store(true);
  rx_thread_ = std::thread(&RelayCANSystem::can_rx_loop, this);
  RCLCPP_INFO(logger_, "RelayCANSystem started");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RelayCANSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  running_.store(false);
  if (rx_thread_.joinable())
  {
    rx_thread_.join();
  }
  (void)send_relay_frame(0x00); // Safe off all relays
  close_can_socket();
  RCLCPP_INFO(logger_, "RelayCANSystem stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RelayCANSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back("relay", "relay_mask", &state_relay_mask_);
  return out;
}

std::vector<hardware_interface::CommandInterface> RelayCANSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back("relay", "relay_mask", &cmd_relay_mask_);
  return out;
}

hardware_interface::return_type RelayCANSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  state_relay_mask_ = static_cast<double>(feedback_relay_mask_.load(std::memory_order_relaxed));
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RelayCANSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const uint8_t mask = static_cast<uint8_t>(std::clamp(std::lround(cmd_relay_mask_), 0L, 255L));

  if (shmitf_.ready())
  {
    HwiCommandIfBlock cmd_if{};
    if (shmitf_.read_cmd_if(cmd_if))
    {
      if (cmd_if.relay.relay_mask != mask)
      {
        cmd_if.relay.relay_mask = mask;
        (void)shmitf_.write_cmd_if(cmd_if);
      }
    }
  }

  const uint8_t prev_desired = desired_relay_mask_.load(std::memory_order_relaxed);
  bool mask_changed = false;
  if (mask != prev_desired)
  {
    desired_relay_mask_.store(mask, std::memory_order_relaxed);
    ack_received_.store(false, std::memory_order_relaxed);
    mask_changed = true;
    retry_count_ = 0;
  }

  const uint8_t fb = feedback_relay_mask_.load(std::memory_order_relaxed);
  const auto now = std::chrono::steady_clock::now();

  // ĐỐI CHIẾU: Nếu trạng thái thực tế đã trùng khớp với mask mong muốn (fb == mask) HOẶC đã nhận ACK
  // VÀ mask không thay đổi:
  // DỪNG GỬI NGAY LẬP TỨC để tránh tràn băng thông CAN bus!
  if (!mask_changed && (fb == mask || ack_received_.load(std::memory_order_relaxed)))
  {
    return hardware_interface::return_type::OK;
  }

  const bool retry_due = (last_send_time_ == std::chrono::steady_clock::time_point::min()) ||
                         (now - last_send_time_ >= RETRY_PERIOD);

  if (mask_changed || (retry_due && retry_count_ < MAX_RETRIES))
  {
    if (send_relay_frame(mask))
    {
      last_send_time_ = now;
      if (!mask_changed)
      {
        ++retry_count_;
      }
      if (fb == mask)
      {
        ack_received_.store(true, std::memory_order_relaxed);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

bool RelayCANSystem::open_can_socket()
{
  if (can_sock_ >= 0) return true;

  can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_sock_ < 0) return false;

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, can_port_.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0)
  {
    close_can_socket();
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  struct timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100ms
  setsockopt(can_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (bind(can_sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    close_can_socket();
    return false;
  }
  return true;
}

void RelayCANSystem::close_can_socket()
{
  if (can_sock_ >= 0)
  {
    close(can_sock_);
    can_sock_ = -1;
  }
}

bool RelayCANSystem::send_relay_frame(uint8_t mask)
{
  if (can_sock_ < 0) return false;

  struct can_frame frame{};
  frame.can_id = tx_can_id_ & CAN_SFF_MASK;
  frame.can_dlc = 4;
  frame.data[0] = sequence_++;
  frame.data[1] = mask;
  frame.data[2] = PROTOCOL_VERSION;
  frame.data[3] = 0x00; // status / reserved

  ssize_t sent = ::write(can_sock_, &frame, sizeof(frame));
  return (sent == sizeof(frame));
}

void RelayCANSystem::can_rx_loop()
{
  struct can_frame frame{};
  while (running_.load(std::memory_order_relaxed))
  {
    if (can_sock_ < 0)
    {
      (void)open_can_socket();
      if (can_sock_ < 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
    }

    ssize_t n = ::read(can_sock_, &frame, sizeof(frame));
    if (n == sizeof(frame))
    {
      const uint32_t id = frame.can_id & CAN_SFF_MASK;
      // Nhận ACK từ cả ID cấu hình hoặc 0x580 / 0x501
      if ((id == rx_can_id_ || id == 0x580 || id == 0x501) && frame.can_dlc >= 2)
      {
        const uint8_t fb_mask = frame.data[1];
        feedback_relay_mask_.store(fb_mask, std::memory_order_relaxed);
        if (fb_mask == desired_relay_mask_.load(std::memory_order_relaxed))
        {
          ack_received_.store(true, std::memory_order_relaxed);
        }
      }
    }
  }
}

} // namespace relay_can_interface

PLUGINLIB_EXPORT_CLASS(relay_can_interface::RelayCANSystem,
                       hardware_interface::SystemInterface)
