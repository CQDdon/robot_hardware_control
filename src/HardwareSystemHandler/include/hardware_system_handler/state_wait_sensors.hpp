#pragma once

#include "hardware_system_handler/fsm.hpp"

class WAIT_HOMING;
class FAULT;
class OFFLINE;

class WAIT_SENSORS : public HshFsm {
public:
    void entry() override { 
        motion_allowed_ = false;
        RCLCPP_INFO(logger_,"WAIT SENSORS State");
     }
    void exit() override {}

    void react(EvSensorsOk const&) override {
        transit<WAIT_HOMING>();
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


    template <typename Event>
    void react(Event const&) { }
};
