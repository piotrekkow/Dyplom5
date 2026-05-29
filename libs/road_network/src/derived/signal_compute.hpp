#pragma once

#include "road_network/signal/derived/intergreen_matrix.hpp"

class GraphDerived;
class SignalData;

namespace signal_compute {
IntergreenMatrix intergreenMatrix(const GraphDerived& graphDerived,
                                  const SignalData& signal, NodeId id);
};