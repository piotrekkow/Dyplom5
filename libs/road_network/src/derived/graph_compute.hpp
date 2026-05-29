#pragma once
#include "road_network/graph/derived/conflict_map.hpp"
#include "road_network/graph/derived/stream_intergreen_map.hpp"
#include "road_network/graph/derived/stream_permitted_map.hpp"

class Graph;
class GraphData;

namespace graph_compute {
struct StreamPermittedCtx {
    // (6.2.1.2) Nie dopuszcza się, w przypadku ruchu z kierunku przeciwnego,
    // stosowania sygnału ogólnego dla skręcających w lewo, jeżeli dla pojazdów
    // skręcających przeznaczone są co najmniej dwa pasy ruchu.
    int permMaxTurnLaneMovementLaneCount = 1;

    // cos(3π/4): angle between headings must exceed 135° to be opposing
    float kOpposingThreshold = -0.70711f;
    // cos(π/4): angle between entry heading and crosswalk direction must be
    // under 45° to be parallel (absolute value handles both line orientations)
    float kParallelThreshold = 0.70711f;
};

ConflictMap conflictMap(const GraphData& data, NodeId id);
StreamIntergreenMap streamIntergreenMap(const Graph& graph, NodeId id);
StreamPermittedMap streamPermittedMap(const Graph& graph, NodeId id,
                                      StreamPermittedCtx ctx = {});
}  // namespace graph_compute