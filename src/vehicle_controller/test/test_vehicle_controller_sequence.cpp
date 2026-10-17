#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_controller/vehicle_controller.hpp"
#include "vehicle_msgs/msg/vehicle_function_command.hpp"
#include "vehicle_msgs/msg/vehicle_motion_command.hpp"

namespace vehicle_controller
{

class VehicleControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    controller_ = std::make_unique<VehicleController>();

    // Initialize controller node
    ASSERT_EQ(controller_->init("test_vehicle_controller"), controller_interface::return_type::OK);

    // Set parameters
    auto node = controller_->get_node();
    node->set_parameter(rclcpp::Parameter("brake_joints", std::vector<std::string>{"RearRightDriver_Joint"}));
    node->set_parameter(rclcpp::Parameter("wheel_joints", std::vector<std::string>{"RearRightWheel_Joint"}));
    node->set_parameter(rclcpp::Parameter("relay_direction_delay_ms", 100));
    node->set_parameter(rclcpp::Parameter("relay_main_delay_ms", 100));
    node->set_parameter(rclcpp::Parameter("relay_deadtime_ms", 100));
    node->set_parameter(rclcpp::Parameter("command_timeout_ms", 500));
    node->set_parameter(rclcpp::Parameter("require_function_command", true));

    // Configure lifecycle
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::SUCCESS);

    // Prepare hardware interfaces
    cmd_interfaces_.emplace_back("RearRightDriver_Joint", "position", &cmd_brake_pos_);
    cmd_interfaces_.emplace_back("RearRightDriver_Joint", "idle", &cmd_brake_idle_);
    cmd_interfaces_.emplace_back("RearRightWheel_Joint", "position", &cmd_wheel_pos_);
    cmd_interfaces_.emplace_back("RearRightWheel_Joint", "idle", &cmd_wheel_idle_);
    cmd_interfaces_.emplace_back("relay", "relay_mask", &cmd_relay_mask_);
    cmd_interfaces_.emplace_back("rs485", "throttle_voltage", &cmd_throttle_voltage_);

    state_interfaces_.emplace_back("RearRightDriver_Joint", "position", &state_brake_pos_);
    state_interfaces_.emplace_back("RearRightDriver_Joint", "velocity", &state_brake_vel_);
    state_interfaces_.emplace_back("RearRightWheel_Joint", "position", &state_wheel_pos_);
    state_interfaces_.emplace_back("RearRightWheel_Joint", "velocity", &state_wheel_vel_);
    state_interfaces_.emplace_back("system", "online", &state_online_);
    state_interfaces_.emplace_back("system", "fault", &state_fault_);
    state_interfaces_.emplace_back("system", "motion_allowed", &state_motion_allowed_);
    state_interfaces_.emplace_back("relay", "relay_mask", &state_relay_mask_);
    state_interfaces_.emplace_back("rs485", "steering_pot_percent", &state_steering_pot_);
    state_interfaces_.emplace_back("system", "vehicle_speed_mps", &state_vehicle_speed_mps_);
    state_interfaces_.emplace_back("system", "battery_voltage", &state_battery_voltage_);

    std::vector<hardware_interface::LoanedCommandInterface> loaned_cmds;
    for (auto & iface : cmd_interfaces_) {
      loaned_cmds.emplace_back(iface);
    }
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (auto & iface : state_interfaces_) {
      loaned_states.emplace_back(iface);
    }

    controller_->assign_interfaces(std::move(loaned_cmds), std::move(loaned_states));
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), controller_interface::CallbackReturn::SUCCESS);

    // Base hardware states
    state_online_ = 1.0;
    state_fault_ = 0.0;
    state_motion_allowed_ = 1.0;
  }

  void TearDown() override
  {
    controller_->on_deactivate(rclcpp_lifecycle::State{});
    rclcpp::shutdown();
  }

  void send_motion(uint8_t gear, double speed, const rclcpp::Time & t, double brake = 0.0, double steer = 0.0)
  {
    VehicleController::MotionInput input;
    input.received = true;
    input.received_ns = t.nanoseconds();
    input.steering_angle_rad = steer;
    input.speed_percentage = speed;
    input.brake_percentage = brake;
    input.gear = gear;
    input.valid = true;
    controller_->motion_buffer_.writeFromNonRT(input);
  }

  void send_function(bool motion_enable, const rclcpp::Time & t, bool emergency_stop = false, uint8_t aux_mask = 0)
  {
    VehicleController::FunctionInput input;
    input.received = true;
    input.received_ns = t.nanoseconds();
    input.motion_enable = motion_enable;
    input.emergency_stop = emergency_stop;
    input.io_output_mask = aux_mask;
    controller_->function_buffer_.writeFromNonRT(input);
  }

  double current_steering_turns() const
  {
    return controller_->current_steering_turns_;
  }

  uint8_t system_telemetry_io_mask() const
  {
    return controller_->system_rt_pub_ ? controller_->system_rt_pub_->msg_.io_output_mask : 0;
  }

  bool system_telemetry_safe() const
  {
    return controller_->system_rt_pub_ ? controller_->system_rt_pub_->msg_.safe_command_active : false;
  }

  std::unique_ptr<VehicleController> controller_;

  std::vector<hardware_interface::CommandInterface> cmd_interfaces_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  double cmd_brake_pos_{0.0};
  double cmd_brake_idle_{1.0};
  double cmd_wheel_pos_{0.0};
  double cmd_wheel_idle_{1.0};
  double cmd_relay_mask_{0.0};
  double cmd_throttle_voltage_{0.0};

  double state_brake_pos_{0.0};
  double state_brake_vel_{0.0};
  double state_wheel_pos_{0.0};
  double state_wheel_vel_{0.0};
  double state_online_{1.0};
  double state_fault_{0.0};
  double state_motion_allowed_{1.0};
  double state_relay_mask_{0.0};
  double state_steering_pot_{50.0};
  double state_vehicle_speed_mps_{0.0};
  double state_battery_voltage_{48.0};
};

TEST_F(VehicleControllerTest, SafeStateGating)
{
  // Startup: no commands sent yet. Throttle must be 0, relays 0.
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);
  EXPECT_EQ(controller_->update(t0, rclcpp::Duration::from_seconds(0.005)), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_), 0);
}

TEST_F(VehicleControllerTest, ForwardDriveSequence)
{
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.005);

  // Enable motion from teleop and request Drive with 50% throttle
  send_function(true, t0, false, 0x05);  // Aux relays 1 and 3 active
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 50.0, t0);

  // Cycle 1: at t0 = 1000.0s. Should enter DIRECTION_ENGAGING immediately.
  // Relay 6 (bit 5: 0x20) must be ON. Relay 8 must be OFF. Throttle MUST be 0.0V!
  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);
  uint8_t mask1 = static_cast<uint8_t>(cmd_relay_mask_);
  EXPECT_EQ(mask1 & 0xE0, 0x20);       // Relay 6 ON, Relay 7 OFF, Relay 8 OFF
  EXPECT_EQ(mask1 & 0x1F, 0x05);       // Aux relays preserved
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0); // No throttle yet!

  // Cycle 2: at t = 1000.05s (50ms later, less than 100ms delay).
  // Relay 6 still ON, Relay 8 still OFF, Throttle still 0.0V.
  const rclcpp::Time t50ms(1000, 50'000'000, RCL_ROS_TIME);
  send_function(true, t50ms, false, 0x05);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 50.0, t50ms);
  EXPECT_EQ(controller_->update(t50ms, dt), controller_interface::return_type::OK);
  uint8_t mask2 = static_cast<uint8_t>(cmd_relay_mask_);
  EXPECT_EQ(mask2 & 0xE0, 0x20);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);

  // Cycle 3: at t = 1000.10s (100ms elapsed).
  // Direction delay met! Should transition to MAIN_ENGAGING.
  // Relay 6 ON and Relay 8 (0x80) ON -> 0xA0. Throttle MUST STILL be 0.0V!
  const rclcpp::Time t100ms(1000, 100'000'000, RCL_ROS_TIME);
  send_function(true, t100ms, false, 0x05);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 50.0, t100ms);
  EXPECT_EQ(controller_->update(t100ms, dt), controller_interface::return_type::OK);
  uint8_t mask3 = static_cast<uint8_t>(cmd_relay_mask_);
  EXPECT_EQ(mask3 & 0xE0, 0xA0);      // Relay 6 + Relay 8 ON
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0); // Still 0.0V!

  // Cycle 4: at t = 1000.15s (150ms). Still in MAIN_ENGAGING.
  const rclcpp::Time t150ms(1000, 150'000'000, RCL_ROS_TIME);
  send_function(true, t150ms, false, 0x05);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 50.0, t150ms);
  EXPECT_EQ(controller_->update(t150ms, dt), controller_interface::return_type::OK);
  uint8_t mask4 = static_cast<uint8_t>(cmd_relay_mask_);
  EXPECT_EQ(mask4 & 0xE0, 0xA0);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);

  // Cycle 5: at t = 1000.20s (200ms elapsed, 100ms in MAIN_ENGAGING).
  // Main delay met! Transitions to RUNNING. Throttle is NOW applied!
  const rclcpp::Time t200ms(1000, 200'000'000, RCL_ROS_TIME);
  send_function(true, t200ms, false, 0x05);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 50.0, t200ms);
  EXPECT_EQ(controller_->update(t200ms, dt), controller_interface::return_type::OK);
  uint8_t mask5 = static_cast<uint8_t>(cmd_relay_mask_);
  EXPECT_EQ(mask5 & 0xE0, 0xA0);
  EXPECT_NEAR(cmd_throttle_voltage_, 2.5, 1e-4); // 50% of 5.0V = 2.5V!
}

TEST_F(VehicleControllerTest, ReverseSwitchingWithDeadtime)
{
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.005);

  // Bring to RUNNING in Forward (Drive)
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);
  const rclcpp::Time t100(1000, 100'000'000, RCL_ROS_TIME);
  const rclcpp::Time t200(1000, 200'000'000, RCL_ROS_TIME);
  send_function(true, t0, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 40.0, t0);
  controller_->update(t0, dt);

  send_function(true, t100, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 40.0, t100);
  controller_->update(t100, dt);

  send_function(true, t200, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 40.0, t200);
  controller_->update(t200, dt);
  EXPECT_NEAR(cmd_throttle_voltage_, 2.0, 1e-4);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0xA0);

  // Now driver shifts to REVERSE at t = 1000.30s with 60% throttle
  const rclcpp::Time t300ms(1000, 300'000'000, RCL_ROS_TIME);
  send_function(true, t300ms, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 60.0, t300ms);
  EXPECT_EQ(controller_->update(t300ms, dt), controller_interface::return_type::OK);

  // Throttle MUST immediately cut to 0.0V and all drive relays must be OFF (DISENGAGING_FOR_REVERSE)
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0x00);

  // During deadtime (e.g. at +50ms = t = 350ms): Still 0V and 0x00
  const rclcpp::Time t350ms(1000, 350'000'000, RCL_ROS_TIME);
  send_function(true, t350ms, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 60.0, t350ms);
  EXPECT_EQ(controller_->update(t350ms, dt), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0x00);

  // After deadtime (100ms at t = 400ms): Now enters DIRECTION_ENGAGING with Relay 7 (0x40)!
  const rclcpp::Time t400ms(1000, 400'000'000, RCL_ROS_TIME);
  send_function(true, t400ms, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 60.0, t400ms);
  EXPECT_EQ(controller_->update(t400ms, dt), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0x40); // Relay 7 ON! Relay 6 OFF! Relay 8 OFF!

  // At t = 500ms: MAIN_ENGAGING with Relay 7 + Relay 8 = 0xC0
  const rclcpp::Time t500ms(1000, 500'000'000, RCL_ROS_TIME);
  send_function(true, t500ms, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 60.0, t500ms);
  EXPECT_EQ(controller_->update(t500ms, dt), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0xC0);

  // At t = 600ms: RUNNING! Throttle 60% of 5V = 3.0V applied
  const rclcpp::Time t600ms(1000, 600'000'000, RCL_ROS_TIME);
  send_function(true, t600ms, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 60.0, t600ms);
  EXPECT_EQ(controller_->update(t600ms, dt), controller_interface::return_type::OK);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0xC0);
  EXPECT_NEAR(cmd_throttle_voltage_, 3.0, 1e-4);
}

TEST_F(VehicleControllerTest, EmergencyStopImmediateCutoff)
{
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.005);

  // Bring to RUNNING
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);
  const rclcpp::Time t100(1000, 100'000'000, RCL_ROS_TIME);
  const rclcpp::Time t200(1000, 200'000'000, RCL_ROS_TIME);
  const rclcpp::Time t250(1000, 250'000'000, RCL_ROS_TIME);

  send_function(true, t0, false, 0x15);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 80.0, t0);
  controller_->update(t0, dt);

  send_function(true, t100, false, 0x15);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 80.0, t100);
  controller_->update(t100, dt);

  send_function(true, t200, false, 0x15);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 80.0, t200);
  controller_->update(t200, dt);
  EXPECT_NEAR(cmd_throttle_voltage_, 4.0, 1e-4);

  // Trigger E-Stop
  send_function(false, t250, true, 0x15);
  EXPECT_EQ(controller_->update(t250, dt), controller_interface::return_type::OK);

  // Throttle must be zero, drive relays must be cutoff, aux relays preserved, brake forced 100%
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0x1F, 0x15);
  EXPECT_DOUBLE_EQ(cmd_brake_idle_, 0.0); // Engaged
}

TEST_F(VehicleControllerTest, SteeringAngleContinuityAndClamping)
{
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.01);
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);
  send_function(true, t0, false, 0);

  // 1. Nominal angle: 0.5 rad -> target turns = (0.5 / 1.0) * 70 = 35 turns
  rclcpp::Time current_t = t0;
  for (int i = 0; i < 100; ++i) {
    current_t += dt;
    send_function(true, current_t, false, 0);
    send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_NEUTRAL, 0.0, current_t, 0.0, 0.5);
    controller_->update(current_t, dt);
  }
  EXPECT_NEAR(current_steering_turns(), 35.0, 0.1);
  EXPECT_NEAR(cmd_wheel_pos_, 35.0 * 2.0 * M_PI, 0.1 * 2.0 * M_PI);
  EXPECT_DOUBLE_EQ(cmd_wheel_idle_, 0.0); // ClosedLoop

  // 2. Full left: 1.0 rad -> reaches 70.0 turns
  for (int i = 0; i < 100; ++i) {
    current_t += dt;
    send_function(true, current_t, false, 0);
    send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_NEUTRAL, 0.0, current_t, 0.0, 1.0);
    controller_->update(current_t, dt);
  }
  EXPECT_NEAR(current_steering_turns(), 70.0, 0.01);

  // 3. CLIFF BUG CHECK: Send 1.05 rad (> 1.0 rad).
  // Target MUST remain clamped at 70.0 turns, NOT collapse to 0.16 turns!
  for (int i = 0; i < 20; ++i) {
    current_t += dt;
    send_function(true, current_t, false, 0);
    send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_NEUTRAL, 0.0, current_t, 0.0, 1.05);
    controller_->update(current_t, dt);
  }
  EXPECT_NEAR(current_steering_turns(), 70.0, 0.01);
  EXPECT_NEAR(cmd_wheel_pos_, 70.0 * 2.0 * M_PI, 0.01);

  // 4. Reverse direction: -1.5 rad (clamped at -70.0 turns)
  for (int i = 0; i < 450; ++i) {
    current_t += dt;
    send_function(true, current_t, false, 0);
    send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_NEUTRAL, 0.0, current_t, 0.0, -1.5);
    controller_->update(current_t, dt);
  }
  EXPECT_NEAR(current_steering_turns(), -70.0, 0.01);
  EXPECT_NEAR(cmd_wheel_pos_, -70.0 * 2.0 * M_PI, 0.01);
}

TEST_F(VehicleControllerTest, GearShiftSpeedInterlock)
{
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.005);

  // Bring to RUNNING in Forward (Drive)
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);
  const rclcpp::Time t100(1000, 100'000'000, RCL_ROS_TIME);
  const rclcpp::Time t200(1000, 200'000'000, RCL_ROS_TIME);
  send_function(true, t0, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 40.0, t0);
  controller_->update(t0, dt);
  send_function(true, t100, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 40.0, t100);
  controller_->update(t100, dt);
  send_function(true, t200, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_DRIVE, 40.0, t200);
  controller_->update(t200, dt);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0xA0); // Forward + Main

  // Simulate vehicle moving at 2.5 rad/s wheel speed
  state_wheel_vel_ = 2.5;

  // Driver shifts to REVERSE at t = 300ms
  const rclcpp::Time t300(1000, 300'000'000, RCL_ROS_TIME);
  send_function(true, t300, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 50.0, t300);
  controller_->update(t300, dt);

  // Disengaging: relays off, throttle 0
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0x00);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);

  // Advance time past deadtime_ms (100ms) to t = 450ms, but wheel is STILL moving!
  const rclcpp::Time t450(1000, 450'000'000, RCL_ROS_TIME);
  send_function(true, t450, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 50.0, t450);
  controller_->update(t450, dt);

  // Interlock active: reverse relay MUST NOT engage while wheel is rolling!
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0x00);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);

  // Now vehicle stops (wheel speed drops below threshold <= 0.1 rad/s)
  state_wheel_vel_ = 0.0;
  const rclcpp::Time t455(1000, 455'000'000, RCL_ROS_TIME);
  send_function(true, t455, false, 0);
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_REVERSE, 50.0, t455);
  controller_->update(t455, dt);

  // Direction delay starts now! Relay 7 (Reverse: 0x40) engages!
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0xE0, 0x40);
  EXPECT_DOUBLE_EQ(cmd_throttle_voltage_, 0.0);
}

TEST_F(VehicleControllerTest, AuxRelayStateTelemetryIntegrity)
{
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.005);
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);

  // In safe mode (motion disabled, aux relays active: headlights 0x01, horn 0x04 -> 0x05)
  state_relay_mask_ = 0x05;
  send_function(false, t0, false, 0x05); // safe mode because motion_enable = false

  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);
  EXPECT_EQ(static_cast<uint8_t>(cmd_relay_mask_) & 0x1F, 0x05);

  // Verify telemetry state: io_output_mask must accurately reflect hardware relay state (0x05), NOT 0!
  EXPECT_EQ(system_telemetry_io_mask(), 0x05);
  EXPECT_TRUE(system_telemetry_safe());
}

TEST_F(VehicleControllerTest, SteeringPotentiometerSoftLimiter)
{
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.01);
  const rclcpp::Time t0(1000, 0, RCL_ROS_TIME);
  send_function(true, t0, false, 0);

  // Center pot: 43.1%. Right bound 85% is 43.1 + 0.85 * 30.6 = 69.11%.
  // Set pot to 70.0% (exceeding +85% right limit)
  state_steering_pot_ = 70.0;
  state_wheel_pos_ = 0.0;

  // Requesting positive turns (right) should be blocked by soft limiter
  send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_NEUTRAL, 0.0, t0, 0.0, 0.5);
  controller_->update(t0, dt);
  EXPECT_NEAR(current_steering_turns(), 0.0, 1e-4);

  // But requesting negative turns (steering back left away from limit) should be permitted!
  rclcpp::Time t1(1000, 50'000'000, RCL_ROS_TIME);
  for (int i = 0; i < 10; ++i) {
    t1 += dt;
    send_function(true, t1, false, 0);
    send_motion(vehicle_msgs::msg::VehicleMotionCommand::GEAR_NEUTRAL, 0.0, t1, 0.0, -0.5);
    controller_->update(t1, dt);
  }
  EXPECT_LT(current_steering_turns(), 0.0);
}

}  // namespace vehicle_controller

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

