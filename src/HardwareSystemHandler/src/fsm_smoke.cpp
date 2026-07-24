#include "hardware_system_handler/fsm.hpp"
#include "hardware_system_handler/events.hpp"
#include "hardware_system_handler/state_init.hpp"
#include "hardware_system_handler/state_wait_sensors.hpp"
#include "hardware_system_handler/state_wait_homing.hpp"
#include "hardware_system_handler/state_ready.hpp"
#include "hardware_system_handler/state_running.hpp"
#include "hardware_system_handler/state_fault.hpp"
#include "hardware_system_handler/state_offline.hpp"
#include "hardware_system_handler/state_stop.hpp"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(logger_, "FSM smoke test: start");
  HardwareSystemHandler::start();

  std::this_thread::sleep_for(200ms);
  RCLCPP_INFO(logger_, "FSM smoke test: EvInitDone");
  HardwareSystemHandler::dispatch(EvInitDone{});

  std::this_thread::sleep_for(200ms);
  RCLCPP_INFO(logger_, "FSM smoke test: EvSensorsOk");
  HardwareSystemHandler::dispatch(EvSensorsOk{});

  std::this_thread::sleep_for(200ms);
  RCLCPP_INFO(logger_, "FSM smoke test: EvHomingDone");
  HardwareSystemHandler::dispatch(EvHomingDone{});

  std::this_thread::sleep_for(200ms);
  RCLCPP_INFO(logger_, "FSM smoke test: EvEnableCmd");
  HardwareSystemHandler::dispatch(EvEnableCmd{});

  std::this_thread::sleep_for(200ms);
  RCLCPP_INFO(logger_, "FSM smoke test: EvAxisError (expect FAULT)");
  HardwareSystemHandler::dispatch(EvAxisError{});

  std::this_thread::sleep_for(200ms);
  RCLCPP_INFO(logger_, "FSM smoke test: done");
  rclcpp::shutdown();
  return 0;
}
