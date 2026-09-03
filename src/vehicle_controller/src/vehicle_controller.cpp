#include "vehicle_controller/vehicle_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/create_publisher.hpp"

namespace vehicle_controller
{
namespace
{
constexpr uint8_t kNeutral = vehicle_msgs::msg::VehicleMotionCommand::GEAR_NEUTRAL;
constexpr uint8_t kDrive = vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE;
constexpr uint8_t kReverse = vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE;

constexpr uint8_t kRelay6Forward = 1 << 5;  // 0x20: Relay 6 Forward
constexpr uint8_t kRelay7Reverse = 1 << 6;  // 0x40: Relay 7 Reverse
constexpr uint8_t kRelay8Main    = 1 << 7;  // 0x80: Relay 8 Main/Power
constexpr uint8_t kDriveRelayMask = kRelay6Forward | kRelay7Reverse | kRelay8Main;  // 0xE0
constexpr uint8_t kAuxRelayMask = static_cast<uint8_t>(~kDriveRelayMask);  // 0x1F: Relays 1..5 preserved
}  // namespace

controller_interface::CallbackReturn VehicleController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("brake_joints", {"RearRightDriver_Joint"});
    auto_declare<std::vector<std::string>>("wheel_joints", {"RearRightWheel_Joint"});
    auto_declare<double>("position_max_offset_turns", position_max_offset_turns_);
    auto_declare<double>("max_steering_angle_rad", max_steering_angle_rad_);
    auto_declare<double>("max_wheel_velocity_rad_s", max_wheel_velocity_rad_s_);
    auto_declare<int>("command_timeout_ms", command_timeout_ms_);
    auto_declare<bool>("require_function_command", require_function_command_);
    auto_declare<int>("relay_direction_delay_ms", relay_direction_delay_ms_);
    auto_declare<int>("relay_main_delay_ms", relay_main_delay_ms_);
    auto_declare<int>("relay_deadtime_ms", relay_deadtime_ms_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot declare parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration VehicleController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : brake_joints_) config.names.push_back(joint + "/position");
  for (const auto & joint : brake_joints_) config.names.push_back(joint + "/idle");
  for (const auto & joint : wheel_joints_) config.names.push_back(joint + "/velocity");
  config.names.push_back("relay/relay_mask");
  config.names.push_back("rs485/throttle_voltage");
  return config;
}

controller_interface::InterfaceConfiguration VehicleController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : brake_joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  for (const auto & joint : wheel_joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  config.names.insert(config.names.end(), {"system/online", "system/fault", "system/motion_allowed", "relay/relay_mask", "rs485/steering_pot_percent"});
  return config;
}

controller_interface::CallbackReturn VehicleController::on_configure(const rclcpp_lifecycle::State &)
{
  brake_joints_ = get_node()->get_parameter("brake_joints").as_string_array();
  wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
  position_max_offset_turns_ = get_node()->get_parameter("position_max_offset_turns").as_double();
  max_steering_angle_rad_ = get_node()->get_parameter("max_steering_angle_rad").as_double();
  max_wheel_velocity_rad_s_ = get_node()->get_parameter("max_wheel_velocity_rad_s").as_double();
  command_timeout_ms_ = get_node()->get_parameter("command_timeout_ms").as_int();
  require_function_command_ = get_node()->get_parameter("require_function_command").as_bool();
  relay_direction_delay_ms_ = get_node()->get_parameter("relay_direction_delay_ms").as_int();
  relay_main_delay_ms_ = get_node()->get_parameter("relay_main_delay_ms").as_int();
  relay_deadtime_ms_ = get_node()->get_parameter("relay_deadtime_ms").as_int();
  if (brake_joints_.empty() || wheel_joints_.empty() || !std::isfinite(position_max_offset_turns_) ||
      position_max_offset_turns_ <= 0.0 ||
      !std::isfinite(max_wheel_velocity_rad_s_) ||
      max_wheel_velocity_rad_s_ <= 0.0 || command_timeout_ms_ <= 0 ||
      relay_direction_delay_ms_ < 0 || relay_main_delay_ms_ < 0 || relay_deadtime_ms_ < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid brake/wheel/relay controller parameters");
    return controller_interface::CallbackReturn::ERROR;
  }

  motion_sub_ = get_node()->create_subscription<vehicle_msgs::msg::VehicleMotionCommand>(
    "/vehicle/control/motion_cmd", rclcpp::QoS(1),
    std::bind(&VehicleController::motion_callback, this, std::placeholders::_1));
  function_sub_ = get_node()->create_subscription<vehicle_msgs::msg::VehicleFunctionCommand>(
    "/vehicle/control/func_cmd", rclcpp::QoS(1),
    std::bind(&VehicleController::function_callback, this, std::placeholders::_1));
  motion_pub_ = rclcpp::create_publisher<vehicle_msgs::msg::VehicleMotionState>(*get_node(), "/vehicle/state/motion", rclcpp::QoS(10));
  system_pub_ = rclcpp::create_publisher<vehicle_msgs::msg::VehicleSystemState>(*get_node(), "/vehicle/state/system", rclcpp::QoS(10));
  motion_rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<vehicle_msgs::msg::VehicleMotionState>>(motion_pub_);
  system_rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<vehicle_msgs::msg::VehicleSystemState>>(system_pub_);
  motion_buffer_.initRT(MotionInput{});
  function_buffer_.initRT(FunctionInput{});
  position_reference_rad_.assign(brake_joints_.size(), 0.0);
  position_reference_valid_ = false;
  drive_seq_state_ = DriveSequenceState::DISENGAGED;
  active_direction_gear_ = kNeutral;
  state_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VehicleController::on_activate(const rclcpp_lifecycle::State &)
{
  drive_seq_state_ = DriveSequenceState::DISENGAGED;
  active_direction_gear_ = kNeutral;
  state_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  write_safe_command();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VehicleController::on_deactivate(const rclcpp_lifecycle::State &)
{
  drive_seq_state_ = DriveSequenceState::DISENGAGED;
  active_direction_gear_ = kNeutral;
  state_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  write_safe_command();
  return controller_interface::CallbackReturn::SUCCESS;
}

void VehicleController::motion_callback(const vehicle_msgs::msg::VehicleMotionCommand::SharedPtr msg)
{
  MotionInput input;
  input.received = true;
  input.received_ns = get_node()->now().nanoseconds();
  input.steering_angle_rad = msg->steering_angle_rad;
  input.speed_percentage = msg->speed_percentage;
  input.brake_percentage = msg->brake_percentage;
  input.gear = msg->gear;
  input.valid = std::isfinite(input.steering_angle_rad) && std::isfinite(input.speed_percentage) &&
    std::isfinite(input.brake_percentage) && input.speed_percentage >= 0.0 && input.speed_percentage <= 100.0 &&
    input.brake_percentage >= 0.0 && input.brake_percentage <= 100.0 &&
    (input.gear == kNeutral || input.gear == kDrive || input.gear == kReverse);
  motion_buffer_.writeFromNonRT(input);
}

void VehicleController::function_callback(const vehicle_msgs::msg::VehicleFunctionCommand::SharedPtr msg)
{
  FunctionInput input;
  input.received = true;
  input.received_ns = get_node()->now().nanoseconds();
  input.motion_enable = msg->motion_enable;
  input.emergency_stop = msg->emergency_stop;
  input.io_output_mask = msg->io_output_mask;
  function_buffer_.writeFromNonRT(input);
}

bool VehicleController::is_timed_out(int64_t received_ns, const rclcpp::Time & time) const
{
  return received_ns <= 0 || time.nanoseconds() < received_ns ||
    (time.nanoseconds() - received_ns) > static_cast<int64_t>(command_timeout_ms_) * 1000000LL;
}

void VehicleController::set_position_command(double position_percentage, bool force_engaged,
                                             bool /*clear_reference_on_release*/)
{
  const size_t brake_count = brake_joints_.size();
  const size_t idle_offset = brake_count;
  if (command_interfaces_.size() < 2 * brake_count || state_interfaces_.size() < 2 * brake_count) return;
  const double requested_percentage = std::clamp(position_percentage, 0.0, 100.0);
  const bool release = !force_engaged && requested_percentage <= 0.0;
  if (release) {
    position_reference_valid_ = false;
    for (size_t i = 0; i < brake_count; ++i) {
      command_interfaces_[i].set_value(0.0);
      command_interfaces_[idle_offset + i].set_value(1.0);
    }
    return;
  }

  const double requested_percentage_val = force_engaged ? 100.0 : requested_percentage;
  if (!position_reference_valid_) {
    for (size_t i = 0; i < brake_count; ++i) {
      position_reference_rad_[i] = state_interfaces_[2 * i].get_value();
    }
    position_reference_valid_ = true;
  }
  const double offset_rad = position_max_offset_turns_ * requested_percentage_val / 100.0 * 2.0 * M_PI;
  for (size_t i = 0; i < brake_count; ++i) {
    command_interfaces_[i].set_value(position_reference_rad_[i] + offset_rad);
    command_interfaces_[idle_offset + i].set_value(0.0);
  }
}

void VehicleController::write_safe_command()
{
  const size_t brake_count = brake_joints_.size();
  const size_t wheel_count = wheel_joints_.size();
  const size_t wheel_offset = 2 * brake_count;
  const size_t relay_offset = wheel_offset + wheel_count;
  const size_t throttle_offset = relay_offset + 1;
  if (command_interfaces_.size() < throttle_offset + 1 || state_interfaces_.size() < 2 * brake_count) return;
  set_position_command(0.0, false, false);
  for (size_t i = 0; i < wheel_count; ++i) command_interfaces_[wheel_offset + i].set_value(0.0);
  command_interfaces_[throttle_offset].set_value(0.0);

  // Preserve auxiliary relays (bits 0..4) while clearing drive relays (bits 5..7)
  const double current_relay = command_interfaces_[relay_offset].get_value();
  const uint8_t aux_mask = static_cast<uint8_t>(std::clamp(std::lround(current_relay), 0L, 255L)) & kAuxRelayMask;
  command_interfaces_[relay_offset].set_value(static_cast<double>(aux_mask));
}

controller_interface::return_type VehicleController::update(const rclcpp::Time & time, const rclcpp::Duration &)
{
  const size_t brake_count = brake_joints_.size();
  const size_t wheel_count = wheel_joints_.size();
  const size_t wheel_offset = 2 * brake_count;
  const size_t relay_offset = wheel_offset + wheel_count;
  const size_t throttle_offset = relay_offset + 1;
  const size_t system_state_index = 2 * (brake_count + wheel_count);
  if (command_interfaces_.size() < throttle_offset + 1 || state_interfaces_.size() < system_state_index + 4) return controller_interface::return_type::ERROR;
  const MotionInput motion = *motion_buffer_.readFromRT();
  const FunctionInput function = *function_buffer_.readFromRT();
  const bool timed_out = is_timed_out(motion.received_ns, time) ||
    (require_function_command_ && is_timed_out(function.received_ns, time));
  const bool online = state_interfaces_[system_state_index].get_value() > 0.5;
  const bool fault = state_interfaces_[system_state_index + 1].get_value() > 0.5;
  const bool hardware_motion_allowed = state_interfaces_[system_state_index + 2].get_value() > 0.5;
  const bool function_requests_stop = function.received &&
    (function.emergency_stop || !function.motion_enable);
  const bool safe = !motion.received || !motion.valid ||
    (require_function_command_ && !function.received) || timed_out || function_requests_stop ||
    !hardware_motion_allowed || !online || fault;

  const uint8_t aux_relay_mask = function.io_output_mask & kAuxRelayMask;
  uint8_t drive_relay_bits = 0;
  double throttle_voltage = 0.0;

  if (safe) {
    drive_seq_state_ = DriveSequenceState::DISENGAGED;
    active_direction_gear_ = kNeutral;
    drive_relay_bits = 0;
    throttle_voltage = 0.0;

    if (function.received && function.emergency_stop) {
      // E-Stop khẩn cấp: Ép phanh 100% (target = position_reference_rad_ + position_max_offset_turns_ (3 turns))
      set_position_command(100.0, true, false);
      for (size_t i = 0; i < wheel_count; ++i) command_interfaces_[wheel_offset + i].set_value(0.0);
      command_interfaces_[throttle_offset].set_value(0.0);
      command_interfaces_[relay_offset].set_value(0.0);
    } else {
      write_safe_command();
    }
  } else {
    // 1. Phanh ODrive (Position joint):
    // Khi brake_percentage <= 0.0 -> trả về IDLE (thả phanh tự do)
    // Khi brake_percentage > 0.0  -> vào ClosedLoop kéo theo mức %
    set_position_command(motion.brake_percentage, false, false);

    // 2. Vận tốc đánh lái (Steering velocity) cho motor quay bánh lái ODrive:
    const double steering_velocity = motion.steering_angle_rad;
    for (size_t i = 0; i < wheel_count; ++i) command_interfaces_[wheel_offset + i].set_value(steering_velocity);

    // 3. Logic tuần tự hóa Relay truyền động (Relay 6/7 -> Relay 8 -> Nạp ga):
    const uint8_t requested_gear = motion.gear;
    const int64_t dir_delay_ns = static_cast<int64_t>(relay_direction_delay_ms_) * 1000000LL;
    const int64_t main_delay_ns = static_cast<int64_t>(relay_main_delay_ms_) * 1000000LL;
    const int64_t deadtime_ns = static_cast<int64_t>(relay_deadtime_ms_) * 1000000LL;

    auto get_dir_bit = [](uint8_t gear) -> uint8_t {
      if (gear == kDrive) return kRelay6Forward;
      if (gear == kReverse) return kRelay7Reverse;
      return 0;
    };

    switch (drive_seq_state_) {
      case DriveSequenceState::DISENGAGED:
        drive_relay_bits = 0;
        throttle_voltage = 0.0;
        if (requested_gear == kDrive || requested_gear == kReverse) {
          active_direction_gear_ = requested_gear;
          drive_seq_state_ = DriveSequenceState::DIRECTION_ENGAGING;
          state_start_time_ = time;
          drive_relay_bits = get_dir_bit(active_direction_gear_);
        }
        break;

      case DriveSequenceState::DIRECTION_ENGAGING:
        throttle_voltage = 0.0;
        if (requested_gear == kNeutral) {
          drive_seq_state_ = DriveSequenceState::DISENGAGED;
          active_direction_gear_ = kNeutral;
          drive_relay_bits = 0;
        } else if (requested_gear != active_direction_gear_) {
          drive_seq_state_ = DriveSequenceState::DISENGAGING_FOR_REVERSE;
          state_start_time_ = time;
          drive_relay_bits = 0;
        } else {
          drive_relay_bits = get_dir_bit(active_direction_gear_);
          if ((time.nanoseconds() - state_start_time_.nanoseconds()) >= dir_delay_ns) {
            drive_seq_state_ = DriveSequenceState::MAIN_ENGAGING;
            state_start_time_ = time;
            drive_relay_bits |= kRelay8Main;
          }
        }
        break;

      case DriveSequenceState::MAIN_ENGAGING:
        throttle_voltage = 0.0;
        if (requested_gear == kNeutral) {
          drive_seq_state_ = DriveSequenceState::DISENGAGED;
          active_direction_gear_ = kNeutral;
          drive_relay_bits = 0;
        } else if (requested_gear != active_direction_gear_) {
          drive_seq_state_ = DriveSequenceState::DISENGAGING_FOR_REVERSE;
          state_start_time_ = time;
          drive_relay_bits = 0;
        } else {
          drive_relay_bits = get_dir_bit(active_direction_gear_) | kRelay8Main;
          if ((time.nanoseconds() - state_start_time_.nanoseconds()) >= main_delay_ns) {
            drive_seq_state_ = DriveSequenceState::RUNNING;
            throttle_voltage = (std::clamp(static_cast<double>(motion.speed_percentage), 0.0, 100.0) / 100.0) * 5.0;
          }
        }
        break;

      case DriveSequenceState::RUNNING:
        if (requested_gear == kNeutral) {
          drive_seq_state_ = DriveSequenceState::DISENGAGED;
          active_direction_gear_ = kNeutral;
          drive_relay_bits = 0;
          throttle_voltage = 0.0;
        } else if (requested_gear != active_direction_gear_) {
          drive_seq_state_ = DriveSequenceState::DISENGAGING_FOR_REVERSE;
          state_start_time_ = time;
          drive_relay_bits = 0;
          throttle_voltage = 0.0;
        } else {
          drive_relay_bits = get_dir_bit(active_direction_gear_) | kRelay8Main;
          throttle_voltage = (std::clamp(static_cast<double>(motion.speed_percentage), 0.0, 100.0) / 100.0) * 5.0;
        }
        break;

      case DriveSequenceState::DISENGAGING_FOR_REVERSE:
        drive_relay_bits = 0;
        throttle_voltage = 0.0;
        if (requested_gear == kNeutral) {
          drive_seq_state_ = DriveSequenceState::DISENGAGED;
          active_direction_gear_ = kNeutral;
        } else if ((time.nanoseconds() - state_start_time_.nanoseconds()) >= deadtime_ns) {
          active_direction_gear_ = requested_gear;
          drive_seq_state_ = DriveSequenceState::DIRECTION_ENGAGING;
          state_start_time_ = time;
          drive_relay_bits = get_dir_bit(active_direction_gear_);
        }
        break;
    }

    const uint8_t final_relay_mask = aux_relay_mask | drive_relay_bits;
    command_interfaces_[relay_offset].set_value(static_cast<double>(final_relay_mask));
    command_interfaces_[throttle_offset].set_value(throttle_voltage);
  }
  publish_states(time, motion, function, timed_out, safe);
  return controller_interface::return_type::OK;
}

void VehicleController::publish_states(const rclcpp::Time & time, const MotionInput & motion,
                                       const FunctionInput & function, bool timed_out, bool safe)
{
  const size_t brake_count = brake_joints_.size();
  const size_t wheel_count = wheel_joints_.size();
  const size_t wheel_state_offset = 2 * brake_count;
  const size_t system_state_index = 2 * (brake_count + wheel_count);
  double wheel_velocity = 0.0;
  for (size_t i = 0; i < wheel_count; ++i) wheel_velocity += state_interfaces_[wheel_state_offset + 2 * i + 1].get_value();
  wheel_velocity /= static_cast<double>(wheel_count);
  if (motion_rt_pub_ && motion_rt_pub_->trylock()) {
    const bool has_steering_pot = state_interfaces_.size() >= system_state_index + 5;
    const float steering_pot_pct = has_steering_pot ? static_cast<float>(state_interfaces_[system_state_index + 4].get_value()) : 0.0F;

    // Calibrated steering potentiometer:
    // Cận trái = 12.5%, Ở giữa = 43.1%, Cận phải = 73.7% (nửa dải đối xứng = 30.6%)
    constexpr float kPotCenter = 43.1f;
    constexpr float kPotHalfSpan = 30.6f;
    const float normalized_steer = std::clamp((steering_pot_pct - kPotCenter) / kPotHalfSpan, -1.0f, 1.0f);
    const float steering_angle = normalized_steer * static_cast<float>(max_steering_angle_rad_);

    auto & out = motion_rt_pub_->msg_;
    out.header.stamp = time;
    out.steering_angle_rad = steering_angle;
    out.wheel_velocity_rad_s = static_cast<float>(wheel_velocity);
    const bool throttle_active = !safe && (drive_seq_state_ == DriveSequenceState::RUNNING);
    out.speed_percentage = throttle_active ? static_cast<float>(motion.speed_percentage) : 0.0F;
    out.brake_percentage = safe ?
      (function.emergency_stop ? 100.0F : 0.0F) : static_cast<float>(motion.brake_percentage);
    out.gear = safe ? kNeutral : motion.gear;
    out.safe_command_active = safe;
    motion_rt_pub_->unlockAndPublish();
  }
  if (system_rt_pub_ && system_rt_pub_->trylock()) {
    auto & out = system_rt_pub_->msg_;
    out.header.stamp = time;
    out.hardware_online = state_interfaces_[system_state_index].get_value() > 0.5;
    out.hardware_fault = state_interfaces_[system_state_index + 1].get_value() > 0.5;
    out.hardware_motion_allowed = state_interfaces_[system_state_index + 2].get_value() > 0.5;
    out.function_motion_enabled = function.motion_enable;
    out.emergency_stop = function.emergency_stop;
    out.command_timed_out = timed_out;
    out.safe_command_active = safe;
    out.io_output_mask = safe ? 0 : static_cast<uint8_t>(state_interfaces_[system_state_index + 3].get_value());
    system_rt_pub_->unlockAndPublish();
  }
}
}  // namespace vehicle_controller

PLUGINLIB_EXPORT_CLASS(vehicle_controller::VehicleController, controller_interface::ControllerInterface)
