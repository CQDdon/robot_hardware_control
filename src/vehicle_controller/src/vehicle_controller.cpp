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
}  // namespace

controller_interface::CallbackReturn VehicleController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("brake_joints", {"RearRightDriver_Joint"});
    auto_declare<std::vector<std::string>>("wheel_joints", {"RearRightWheel_Joint"});
    auto_declare<double>("spring_brake_travel_rad", spring_brake_travel_rad_);
    auto_declare<double>("max_wheel_velocity_rad_s", max_wheel_velocity_rad_s_);
    auto_declare<int>("command_timeout_ms", command_timeout_ms_);
    auto_declare<bool>("require_function_command", require_function_command_);
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
  config.names.push_back("io/output_mask");
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
  config.names.insert(config.names.end(), {"system/online", "system/fault", "system/motion_allowed", "io/output_mask"});
  return config;
}

controller_interface::CallbackReturn VehicleController::on_configure(const rclcpp_lifecycle::State &)
{
  brake_joints_ = get_node()->get_parameter("brake_joints").as_string_array();
  wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
  spring_brake_travel_rad_ = get_node()->get_parameter("spring_brake_travel_rad").as_double();
  max_wheel_velocity_rad_s_ = get_node()->get_parameter("max_wheel_velocity_rad_s").as_double();
  command_timeout_ms_ = get_node()->get_parameter("command_timeout_ms").as_int();
  require_function_command_ = get_node()->get_parameter("require_function_command").as_bool();
  if (brake_joints_.empty() || wheel_joints_.empty() || !std::isfinite(spring_brake_travel_rad_) ||
      spring_brake_travel_rad_ <= 0.0 || !std::isfinite(max_wheel_velocity_rad_s_) ||
      max_wheel_velocity_rad_s_ <= 0.0 || command_timeout_ms_ <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid brake/wheel controller parameters");
    return controller_interface::CallbackReturn::ERROR;
  }
  brake_reference_position_.assign(brake_joints_.size(), 0.0);
  brake_target_valid_ = false;
  latched_brake_percentage_ = -1.0;

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
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VehicleController::on_activate(const rclcpp_lifecycle::State &)
{
  write_safe_command();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VehicleController::on_deactivate(const rclcpp_lifecycle::State &)
{
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

void VehicleController::set_brake_command(double brake_percentage, bool force_engaged)
{
  const size_t brake_count = brake_joints_.size();
  const size_t idle_offset = brake_count;
  const bool release = !force_engaged && brake_percentage <= 0.0;
  if (release) {
    for (size_t i = 0; i < brake_count; ++i) command_interfaces_[idle_offset + i].set_value(1.0);
    brake_target_valid_ = false;
    latched_brake_percentage_ = -1.0;
    return;
  }

  const double requested_percentage = force_engaged ? 100.0 : brake_percentage;
  if (!brake_target_valid_ || std::abs(requested_percentage - latched_brake_percentage_) > 1e-9) {
    for (size_t i = 0; i < brake_count; ++i) {
      brake_reference_position_[i] = state_interfaces_[2 * i].get_value();
    }
    brake_target_valid_ = true;
    latched_brake_percentage_ = requested_percentage;
  }
  for (size_t i = 0; i < brake_count; ++i) {
    command_interfaces_[i].set_value(brake_reference_position_[i] +
      spring_brake_travel_rad_ * requested_percentage / 100.0);
    command_interfaces_[idle_offset + i].set_value(0.0);
  }
}

void VehicleController::write_safe_command()
{
  const size_t brake_count = brake_joints_.size();
  const size_t wheel_offset = 2 * brake_count;
  const size_t io_offset = wheel_offset + wheel_joints_.size();
  if (command_interfaces_.size() != io_offset + 1 || state_interfaces_.size() < 2 * brake_count) return;
  // The brake actuator is released by default.  A safe command stops the
  // wheel and outputs without changing the brake axis out of IDLE.
  set_brake_command(0.0, false);
  for (size_t i = 0; i < wheel_joints_.size(); ++i) command_interfaces_[wheel_offset + i].set_value(0.0);
  command_interfaces_[io_offset].set_value(0.0);
}

controller_interface::return_type VehicleController::update(const rclcpp::Time & time, const rclcpp::Duration &)
{
  const size_t brake_count = brake_joints_.size();
  const size_t wheel_count = wheel_joints_.size();
  const size_t wheel_offset = 2 * brake_count;
  const size_t io_offset = wheel_offset + wheel_count;
  const size_t system_state_index = 2 * (brake_count + wheel_count);
  if (command_interfaces_.size() != io_offset + 1 || state_interfaces_.size() != system_state_index + 4) return controller_interface::return_type::ERROR;
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

  if (safe) {
    write_safe_command();
    // E-stop is an explicit brake request. Timeout/startup/normal stop leave
    // the independent brake actuator released.
    if (function.received && function.emergency_stop) {
      set_brake_command(100.0, false);
    }
  } else {
    set_brake_command(motion.brake_percentage, false);
    const double gear_sign = motion.gear == kDrive ? 1.0 : (motion.gear == kReverse ? -1.0 : 0.0);
    // The wheel and brake are two independent actuators. A brake-position
    // request must not alter the wheel velocity command.
    const double wheel_velocity = gear_sign * max_wheel_velocity_rad_s_ *
      motion.speed_percentage / 100.0;
    for (size_t i = 0; i < wheel_count; ++i) command_interfaces_[wheel_offset + i].set_value(wheel_velocity);
    command_interfaces_[io_offset].set_value(static_cast<double>(function.io_output_mask));
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
    auto & out = motion_rt_pub_->msg_;
    out.header.stamp = time;
    // The legacy message has no brake-position field. Keep this field zero;
    // brake command state is represented by brake_percentage below.
    out.steering_angle_rad = 0.0F;
    out.wheel_velocity_rad_s = static_cast<float>(wheel_velocity);
    out.speed_percentage = safe ? 0.0F : static_cast<float>(motion.speed_percentage);
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
