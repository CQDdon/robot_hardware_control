#pragma once

#include "hardware_system_handler/fsm.hpp"
#include "hardware_system_handler/state_ready.hpp"
#include "hardware_system_handler/state_stop.hpp"
#include "hardware_system_handler/state_offline.hpp"

class FAULT : public HshFsm {
public:
    void entry() {
        RCLCPP_INFO(logger_, "FAULT State");
    }
    void exit() {}

    void react(EvClearError const&) {
        transit<READY>();
    }

    void react(EvCriticalError const&) {
        transit<STOP>();
    }

    void react(EvSensorsTimeout const&) {
        transit<FAULT>();
    }

    void react(EvAxisOffline const&) {
        transit<OFFLINE>();
    }

    template <typename Event>
    void react(Event const&) { }
};
