#pragma once

#include <road_network/util/ids.hpp>

class Graph;
class Signal;
class Demand;
class Control;

namespace logs {
void streamPermitted(NodeId node, const Graph& graph);
void streamIntergreen(NodeId node, const Graph& graph);
void intergreenMatrix(NodeId node, const Signal& signal);
void throughput(TimingId timing, const Control& control, const Signal& signal,
                const Graph& graph, const Demand& demand);
void timingDiagram(TimingId timing, const Control& control,
                   const Signal& signal);
}  // namespace logs
