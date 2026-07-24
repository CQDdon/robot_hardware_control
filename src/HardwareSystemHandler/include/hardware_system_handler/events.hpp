#pragma once
#include <tinyfsm.hpp>
#include <atomic>
#include "rclcpp/rclcpp.hpp"

struct EvInitDone : tinyfsm::Event {};
struct EvSensorsOk : tinyfsm::Event {};
struct EvSensorsTimeout : tinyfsm::Event {};
struct EvAxisError : tinyfsm::Event {};
struct EvAxisOffline : tinyfsm::Event {};
struct EvAxisOnline : tinyfsm::Event {};
struct EvAxisOnlineFault : tinyfsm::Event {};
struct EvHomingDone : tinyfsm::Event {};
struct EvHomingTimeout : tinyfsm::Event {};
struct EvHomingRequired : tinyfsm::Event {};
struct EvEnableCmd : tinyfsm::Event {};
struct EvClearError : tinyfsm::Event {};
struct EvCriticalError : tinyfsm::Event {};

inline std::atomic<bool> motion_allowed_{false};
// inline std::atomic<bool> homing_required_{false};

inline rclcpp::Logger logger_{rclcpp::get_logger("HardwareSystemHandler")};
