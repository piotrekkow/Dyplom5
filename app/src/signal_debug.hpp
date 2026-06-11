#pragma once

#include <road_network/control.hpp>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>
#include <road_network/util/ids.hpp>

namespace debug_print {
void intersectionTimings(IntersectionId intersection, const Signal& signal,
                         const Control& control, const Graph& graph,
                         const Demand& demand);

void streamPermitted(NodeId node, const Graph& graph);
void streamIntergreen(NodeId node, const Graph& graph);
void intergreenMatrix(SignalConfigId config, const Signal& signal);
}  // namespace debug_print
