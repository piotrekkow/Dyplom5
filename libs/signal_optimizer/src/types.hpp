#pragma once

#include <common/matrix.hpp>
#include <control/schema/stage_sequence.hpp>
#include <cstdint>
#include <geometry/vector2.hpp>
#include <limits>
#include <regulations/types/turn_kind.hpp>
#include <road_network/demand/demand_data.hpp>
#include <road_network/demand/demand_derived.hpp>
#include <road_network/demand/derived/saturation.hpp>
#include <road_network/util/ids.hpp>
#include <road_network/util/stream_matrix.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using GId = int;
enum class GKind : uint8_t { Protected, Permitted, Crosswalk };

struct GCluster {
    double Q;
    double S;
};

struct GGroup {
    GId id = 0;
    GKind kind = GKind::Protected;
    int entryIdx = -1;   // index into GInput::entries; -1 for crosswalk
    int seriesIdx = -1;  // index into GInput::series; -1 for movement
    std::vector<int>
        clusterIdxs;  // indices local to GInput::EntryData::clusters
    std::vector<GCluster> clusters;
    std::vector<StreamId>
        streams;  // MovementId or CrosswalkId from road_network
    double maxY = 0.0;
    int tMin = 0;
};

// [evac group id][appr group id] = maxIntergreen; -1 == compatible
using GIntergreenMatrix = Matrix<GId, int>;

// [evac group id][appr group id] = bool whether can run simultaneously
using GCompatibilityGraph = std::unordered_map<GId, std::unordered_set<GId>>;

struct GConfig {
    std::vector<GGroup> groups;
    GIntergreenMatrix intergreen;
    GCompatibilityGraph compatibility;
};

struct GStage {
    std::vector<GId> groupIds;
};

struct GGreen {
    std::vector<int> tStage;
    std::vector<int> tGroup;
};

struct GCycle {
    control::StageSequence<GId> steps;
    double D = std::numeric_limits<double>::infinity();
};

struct GInput {
    struct MovementData {
        MovementId id;
        TurnKind turn;
    };
    struct ClusterData {
        ClusterId id;
        std::vector<MovementData> movements;
        Saturation saturation;
        double Q;
    };
    struct EntryData {
        EntryId id;
        Vector2 heading;
        std::vector<ClusterData> clusters;
    };
    struct SeriesData {
        CrosswalkSeriesId id;
        std::vector<CrosswalkId> crosswalks;
        int minGreen;
    };

    std::vector<EntryData> entries;
    std::vector<SeriesData> series;
    StreamMatrix<int> M;
    StreamMatrix<bool> P;
    int T;
};
