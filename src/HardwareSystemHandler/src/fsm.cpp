#include "hardware_system_handler/fsm.hpp"
#include "hardware_system_handler/state_init.hpp"
#include "hardware_system_handler/state_wait_sensors.hpp"
#include "hardware_system_handler/state_wait_homing.hpp"
#include "hardware_system_handler/state_ready.hpp"
#include "hardware_system_handler/state_running.hpp"
#include "hardware_system_handler/state_fault.hpp"
#include "hardware_system_handler/state_offline.hpp"
#include "hardware_system_handler/state_stop.hpp"
#include <iostream>

// Define initial state for the FSM
FSM_INITIAL_STATE(HardwareSystemHandler, INIT)

// Debug: verify the initial state is set correctly
// void test_fsm_init() {
//     std::cout << "[FSM.CPP] Testing FSM initialization..." << std::endl;
//     HardwareSystemHandler::set_initial_state();
//     std::cout << "[FSM.CPP] After set_initial_state" << std::endl;
//     HardwareSystemHandler::enter();
//     std::cout << "[FSM.CPP] After enter" << std::endl;
// }
