#pragma once

#include "hardware_system_handler/fsm.hpp"

// Forward declarations of other states used in transit<>
class READY;
class STOP;
class OFFLINE;

class WAIT_HOMING : public HshFsm {
public:
    void entry() override {
        motion_allowed_ = false;
        RCLCPP_INFO(logger_, "WAIT HOMING State");
    }

    void exit() override {}

    void react(EvHomingDone const&) override {
        transit<READY>();
    }

    void react(EvHomingTimeout const&) override {
        transit<STOP>();
    }

    void react(EvAxisError const&) override {
        transit<STOP>();
    }

    void react(EvAxisOffline const&) override {
        transit<OFFLINE>();
    }

    void react(EvCriticalError const&) override {
        transit<STOP>();
    }

    template <typename Event>
    void react(Event const&) { }
};
