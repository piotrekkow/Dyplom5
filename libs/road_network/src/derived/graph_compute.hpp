#pragma once

#include "road_network/graph/derived/conflict_point.hpp"
#include "road_network/graph/derived/intergreen_diagnostic.hpp"
#include "road_network/util/stream_matrix.hpp"

class Graph;
class GraphData;

namespace graph_compute {

struct StreamIntergreenMaps {
    StreamMatrix<int> max;
    StreamMatrix<IntergreenDiagnostic> diag;
};
StreamMatrix<std::vector<ConflictPoint>> conflictMap(const GraphData& data,
                                                     NodeId id);
StreamIntergreenMaps streamIntergreenMaps(const Graph& graph, NodeId id);
StreamMatrix<bool> streamPermittedMap(const Graph& graph, NodeId id);
StreamMatrix<int> streamApproachMap(const Graph& graph, NodeId id);
}  // namespace graph_compute
