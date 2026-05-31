#pragma once

#include <limits>
#include <road_network/demand/demand_data.hpp>
#include <road_network/demand/demand_derived.hpp>
#include <road_network/graph/derived/stream_intergreen_map.hpp>
#include <road_network/graph/derived/stream_permitted_map.hpp>
#include <road_network/graph/derived/turn_type.hpp>
#include <road_network/util/geometry/vector2.hpp>
#include <road_network/util/ids.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "road_network/demand/derived/saturation.hpp"

using GId = int;
enum class GKind : uint8_t { Protected, Permitted, Crosswalk };

struct GCluster {
    float Q;
    float S;
};

struct GGroup {
    GId id = 0;
    GKind kind = GKind::Protected;
    int entryIdx = -1;   // index into GInput::entries; -1 for crosswalk
    int seriesIdx = -1;  // index into GInput::crosswalkSeries; -1 for movement
    std::vector<int>
        clusterIdxs;  // indices local to GInput::EntryData::clusters
    std::vector<GCluster> clusters;
    std::vector<StreamId>
        streams;            // MovementId or CrosswalkId from road_network
    std::vector<float> ys;  // y coefficients of all clusters
    float maxY = 0.0;
    int tMin = 0;
};

// [evac group id][appr group id] = maxIntergreen
using GIntergreenMatrix = std::vector<std::vector<int>>;

// [evac group id][appr group id] = bool whether can run simultaneously
using GCompatibilityGraph = std::unordered_map<GId, std::unordered_set<GId>>;

// Defines how the movements on each entry are split into signal groups, and
// whether each group is protected or permitted.
struct GConfig {
    std::vector<GGroup> groups;
    GIntergreenMatrix intergreen;
    GCompatibilityGraph compatibility;
};

// Set of signal groups that all display green simultaneously without violating
// conflict or intergreen rules
struct GPhase {
    std::vector<GId> groupIds;
};

enum class GAspect : uint8_t { Red, RedYellow, Green, Yellow, FlashingGreen };

struct GGroupTransition {
    int green = 0;
    GKind kind;
    bool prevActive;
    bool nextActive;
};

// the interleaved period during which the evacuating groups complete their
// clearance (yellow / flashing-green) while the approaching groups wait for
// intergreen to be satisfied before going green
struct GTransition {
    int makespan = 0;
    std::unordered_map<GId, GGroupTransition> groups;
};

// green time allocation for groups and phases
struct GGreen {
    std::vector<int> tPhase;
    std::vector<int> tGroup;
};

struct GCycle {
    std::vector<GPhase> phases;
    std::vector<GTransition> transitions;
    GGreen alloc;
    double D = std::numeric_limits<double>::infinity();
};

struct GInput {
    struct EntryLaneData {
        EntryLaneId id;
        Saturation saturation;
    };
    struct MovementData {
        MovementId id;
        std::vector<EntryLaneData> lanes;
        TurnType turn;
    };
    struct ClusterData {
        ClusterId id;
        std::vector<MovementData> movements;
        std::vector<EntryLaneData> lanes;
        Saturation saturation;
        float Q;
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
    StreamIntergreenMap M;
    StreamPermittedMap P;
    int T;
};