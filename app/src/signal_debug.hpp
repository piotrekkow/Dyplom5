#pragma once

#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>
#include <road_network/util/ids.hpp>

namespace debug_print {
void nodeTimings(NodeId node, const Signal& signal, const Graph& graph,
                 const Demand& demand);

void streamPermitted(NodeId node, const Graph& graph);
void streamIntergreen(NodeId node, const Graph& graph);
void intergreenMatrix(NodeId node, const Signal& signal);
}  // namespace debug_print