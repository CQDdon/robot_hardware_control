#pragma once

#include <tinyfsm.hpp>
#include "events.hpp"

// Base FSM state for Hardware System Handler
struct HardwareSystemHandler : tinyfsm::Fsm<HardwareSystemHandler>
{
  // Default no-op handlers (can be overridden in concrete states)
  virtual void react(EvInitDone const &) { }
  virtual void react(EvSensorsOk const &) { }
  virtual void react(EvSensorsTimeout const &) { }
  virtual void react(EvAxisError const &) { }
  virtual void react(EvAxisOffline const &) { }
  virtual void react(EvAxisOnline const &) { }
  virtual void react(EvAxisOnlineFault const &) { }
  virtual void react(EvHomingDone const &) { }
  virtual void react(EvHomingTimeout const &) { }
  virtual void react(EvHomingRequired const &) { }
  virtual void react(EvEnableCmd const &) { }
  virtual void react(EvClearError const &) { }
  virtual void react(EvCriticalError const &) { }

  // Catch-all for unhandled events
  template <typename Event>
  void react(Event const &) { }

  virtual void entry() { }
  virtual void exit() { }
};

// Convenient alias to refer to the FSM type
using HshFsm = HardwareSystemHandler;
