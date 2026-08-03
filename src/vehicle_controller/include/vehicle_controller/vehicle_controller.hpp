#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_msgs/msg/vehicle_function_command.hpp"
#include "vehicle_msgs/msg/vehicle_motion_command.hpp"
#include "vehicle_msgs/msg/vehicle_motion_state.hpp"
#include "vehicle_msgs/msg/vehicle_system_state.hpp"

namespace vehicle_controller
{
class VehicleController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  struct MotionInput
  {
    bool received{false};
    bool valid{false};
    int64_t received_ns{0};
    double steering_angle_rad{0.0};
    double speed_percentage{0.0};
    double brake_percentage{100.0};
    uint8_t gear{0};
  };
  struct FunctionInput
  {
    bool received{false};
    int64_t received_ns{0};
    bool motion_enable{false};
    bool emergency_stop{false};
    uint8_t io_output_mask{0};
  };

  void motion_callback(const vehicle_msgs::msg::VehicleMotionCommand::SharedPtr msg);
  void function_callback(const vehicle_msgs::msg::VehicleFunctionCommand::SharedPtr msg);
  void write_safe_command();
  void set_brake_command(double brake_percentage, bool force_engaged);
  bool is_timed_out(int64_t received_ns, const rclcpp::Time & time) const;
  void publish_states(const rclcpp::Time & time, const MotionInput & motion,
                      const FunctionInput & function, bool timed_out, bool safe);

  std::vector<std::string> brake_joints_;
  std::vector<std::string> wheel_joints_;
  double spring_brake_travel_rad_{3.14159265359};
  double max_wheel_velocity_rad_s_{20.0};
  int command_timeout_ms_{250};
  bool require_function_command_{true};
  std::vector<double> brake_reference_position_;
  bool brake_target_valid_{false};
  double latched_brake_percentage_{-1.0};

  realtime_tools::RealtimeBuffer<MotionInput> motion_buffer_;
  realtime_tools::RealtimeBuffer<FunctionInput> function_buffer_;
  rclcpp::Subscription<vehicle_msgs::msg::VehicleMotionCommand>::SharedPtr motion_sub_;
  rclcpp::Subscription<vehicle_msgs::msg::VehicleFunctionCommand>::SharedPtr function_sub_;
  rclcpp::Publisher<vehicle_msgs::msg::VehicleMotionState>::SharedPtr motion_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::VehicleSystemState>::SharedPtr system_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<vehicle_msgs::msg::VehicleMotionState>> motion_rt_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<vehicle_msgs::msg::VehicleSystemState>> system_rt_pub_;
};
}  // namespace vehicle_controller
