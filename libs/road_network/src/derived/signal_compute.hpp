#pragma once

#include <unordered_map>

#include "road_network/signal/derived/approach_matrix.hpp"
#include "road_network/signal/derived/intergreen_matrix.hpp"
#include "road_network/signal/derived/protection_resolution.hpp"
#include "road_network/util/ids.hpp"

class GraphDerived;
class GraphData;
class SignalData;
class Signal;
struct Regulations;

namespace signal_compute {
IntergreenMatrix intergreenMatrix(const GraphDerived& graphDerived,
                                  const Signal& signal,
                                  const Regulations& regulations, NodeId id);

ApproachMatrix approachMatrix(const GraphDerived& graphDerived,
                              const Signal& signal, NodeId id,
                              const IntergreenMatrix& intergreen);

std::unordered_map<SignalGroupId, ProtectionResolution> protectionResolution(
    const SignalData& signal, const GraphData& graph,
    const Regulations& regulations, NodeId id);
};  // namespace signal_compute