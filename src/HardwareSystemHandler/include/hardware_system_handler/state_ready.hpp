#pragma once

#include "hardware_system_handler/fsm.hpp"
#include "hardware_system_handler/state_running.hpp"
#include "hardware_system_handler/state_fault.hpp"
#include "hardware_system_handler/state_offline.hpp"
#include "hardware_system_handler/state_wait_homing.hpp"
#include "hardware_system_handler/state_stop.hpp"

class READY : public HshFsm {
public:
    void entry() override {
        RCLCPP_INFO(logger_, "READY State");
    }
    void exit() override {}

    void react(EvEnableCmd const&) override {
        transit<RUNNING>();
    }

    void react(EvSensorsTimeout const&) override {
        transit<FAULT>();
    }
    void react(EvAxisOffline const&) override {
        transit<OFFLINE>();
    }

    void react(EvAxisError const&) override {
        transit<FAULT>();
    }

    void react(EvHomingRequired const&) override {
        transit<WAIT_HOMING>();
    }

    void react(EvCriticalError const&) override {
        transit<STOP>();
    }

    template <typename Event>
    void react(Event const&) { }
};
