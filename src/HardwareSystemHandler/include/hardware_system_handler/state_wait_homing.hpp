#pragma once

#include "hardware_system_handler/fsm.hpp"

// Forward declarations of other states used in transit<>
class READY;
class STOP;
class OFFLINE;

class WAIT_HOMING : public HshFsm {
public:
    void entry() {
        motion_allowed_ = false;
        RCLCPP_INFO(logger_, "WAIT HOMING State");
    }

    void exit() {}

    void react(EvHomingDone const&) {
        transit<READY>();
    }

    void react(EvHomingTimeout const&) {
        transit<STOP>();
    }

    void react(EvAxisError const&) {
        transit<STOP>();
    }

    void react(EvAxisOffline const&) {
        transit<OFFLINE>();
    }

    template <typename Event>
    void react(Event const&) { }
};
