#pragma once

#include "road_network/control/control_derived.hpp"
#include "road_network/util/ids.hpp"

class GraphData;
class SignalData;
class SignalDerived;
class Timing;

namespace control_compute {

ConfigCompleteness configCompleteness(const GraphData& graph,
                                      const SignalData& signal,
                                      SignalConfigId id);

TimingValidity timingValidity(const SignalData& signal,
                              const SignalDerived& signalDerived,
                              const Timing& timing);

}  // namespace control_compute
