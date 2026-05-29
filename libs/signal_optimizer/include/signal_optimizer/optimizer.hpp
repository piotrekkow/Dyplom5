#pragma once

#include <optional>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal/data/sequence.hpp>
#include <road_network/util/ids.hpp>
#include <vector>

namespace signal_optimizer {

struct MovementGroupResult {
    std::vector<ClusterId> clusters;
    std::vector<MovementId> movements;
    bool isProtected;
    Sequence sequence;
};

struct CrosswalkGroupResult {
    CrosswalkSeriesId seriesId;
    std::vector<CrosswalkId> crosswalks;
    Sequence sequence;
};

struct OptimizeResult {
    std::vector<MovementGroupResult> mgResults;
    std::vector<CrosswalkGroupResult> cgResults;
};

std::optional<OptimizeResult> optimize(NodeId node, const Graph& graph,
                                       const Demand& demand, int cycleLength);

}  // namespace signal_optimizer
