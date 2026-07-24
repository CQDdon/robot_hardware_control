#pragma once

#include "hardware_system_handler/fsm.hpp"
#include "hardware_system_handler/state_wait_sensors.hpp"
#include "hardware_system_handler/state_fault.hpp"
#include "hardware_system_handler/state_offline.hpp"

class INIT : public HshFsm {
public:
    void entry() override {
        motion_allowed_ = false;\
        RCLCPP_INFO(logger_, ">>> Entered INIT State <<<");
    }
    void exit() override {}

    void react(EvInitDone const&) override {
        transit<WAIT_SENSORS>();
    }

    void react(EvSensorsTimeout const&) {
        transit<FAULT>();
    }

    void react(EvAxisOffline const&) {
        transit<OFFLINE>();
    }

    void react(EvAxisError const&) {
        transit<FAULT>();
    }

    template <typename Event>
    void react(Event const&) { }
};
