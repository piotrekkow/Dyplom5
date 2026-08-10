#pragma once

#include <control/schema/stage_sequence.hpp>
#include <expected>
#include <functional>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/util/ids.hpp>
#include <vector>

namespace signal_optimizer {

struct MovementGroupResult {
    std::vector<ClusterId> clusters;
    std::vector<MovementId> movements;
    bool isProtected;
    int minGreen;
};

struct CrosswalkGroupResult {
    CrosswalkSeriesId seriesId;
    std::vector<CrosswalkId> crosswalks;
    int minGreen;
};

struct OptimizationStats {
    int configs = 0;
    long sequences = 0;
    long fullyInfeasible = 0;
};

struct OptimizeResult {
    std::vector<MovementGroupResult> mgResults;
    std::vector<CrosswalkGroupResult> cgResults;
    control::StageSequence<int> stages;
    int cycleLength;
    OptimizationStats stats;
};

enum class OptimizeError : uint8_t {
    InvalidNode,
    InvalidCycleLength,
    MissingDemandData,
    NoStreams,
    NoConfigurations,
    NoFeasiblePlan,
    Cancelled,
    InternalFailure,
};

// Runs the full optimization pipeline for a single intersection node.
// Failures are reported explicitly; no partial result is returned.
std::expected<OptimizeResult, OptimizeError> optimize(
    NodeId node, const Graph& graph, const Demand& demand, int cycleLength,
    std::function<bool()> cancelled = {},
    std::function<void(int completed, int total)> progress = {});

}  // namespace signal_optimizer
