#pragma once

#include <optional>
#include <vector>

#include "graph/graph_data.hpp"
#include "graph/graph_derived.hpp"
#include "road_network/graph/data/movement.hpp"
#include "road_network/util/geometry/vector2.hpp"
#include "util/ids.hpp"
#include <functional>

struct EntryLaneSpec {
    float width = 3.5f;
    float stopLineOffset = 0.0f;
    std::optional<float> storageLength = std::nullopt;
};

struct ExitLaneSpec {
    float width = 3.5f;
};

class Graph {
   public:
    const GraphData& data() const { return data_; }
    const GraphDerived& derived() const { return derived_; }

    struct NodeMutation {
        NodeId id;
        std::move_only_function<void()> undo;
    };

    template <typename EntityId>
    struct EntityMutation {
        EntityId id;
        NodeId at;
        std::move_only_function<void()> undo;
    };

    // Node
    NodeMutation createNode(Position center);
    NodeMutation removeNode(NodeId at);

    // Entry
    EntityMutation<EntryId> createEntry(NodeId at, Position position,
                                        Vector2 heading);
    EntityMutation<EntryId> setEntrySpeedLimit(EntryId id, float speedLimit);
    EntityMutation<EntryId> setEntryPosition(EntryId id, Position position);
    EntityMutation<EntryId> setEntryHeading(EntryId id, Vector2 heading);
    NodeMutation removeEntry(EntryId id);

    // Exit
    EntityMutation<ExitId> createExit(NodeId at, Position position,
                                      Vector2 heading);
    EntityMutation<ExitId> setExitSpeedLimit(ExitId id, float speedLimit);
    EntityMutation<ExitId> setExitPosition(ExitId id, Position position);
    EntityMutation<ExitId> setExitHeading(ExitId id, Vector2 heading);
    NodeMutation removeExit(ExitId id);

    // EntryLane / ExitLane — atomic approach-level mutation
    EntityMutation<EntryId> setEntryLanes(EntryId at,
                                          std::vector<EntryLaneSpec> specs);
    EntityMutation<ExitId> setExitLanes(ExitId at,
                                        std::vector<ExitLaneSpec> specs);

    // CrosswalkSeries
    EntityMutation<CrosswalkSeriesId> createCrosswalkSeries(NodeId at);
    EntityMutation<CrosswalkSeriesId> setCrosswalkSeriesPedestrianPath(
        CrosswalkSeriesId id, Polyline path);
    NodeMutation removeCrosswalkSeries(CrosswalkSeriesId id);

    // Crosswalk
    EntityMutation<CrosswalkId> createCrosswalk(CrosswalkSeriesId at,
                                                Line line);
    EntityMutation<CrosswalkId> setCrosswalkLine(CrosswalkId id, Line line);
    EntityMutation<CrosswalkId> setCrosswalkEdge(CrosswalkId id,
                                                 Crosswalk::SegmentParams edge);
    EntityMutation<CrosswalkId> setCrosswalkSplit(
        CrosswalkId id, Crosswalk::SegmentParams split);
    EntityMutation<CrosswalkId> setCrosswalkType(CrosswalkId id,
                                                 CrosswalkType type);
    NodeMutation removeCrosswalk(CrosswalkId id);

    // Movement
    EntityMutation<MovementId> createMovement(EntryId from, ExitId to,
                                              std::vector<EntryLaneId> lanes,
                                              std::vector<int> split);
    EntityMutation<MovementId> setMovementSplits(MovementId id,
                                                 std::vector<int> splits);
    EntityMutation<MovementId> setMovementType(MovementId id,
                                               MovementType type);
    EntityMutation<MovementId> setMovementPathSpec(MovementId id,
                                                   MovementPathSpec spec);
    NodeMutation removeMovement(MovementId id);

    // Cluster
    EntityMutation<ClusterId> createCluster(EntryId at,
                                            std::vector<MovementId> movements,
                                            std::vector<EntryLaneId> lanes);
    NodeMutation removeCluster(ClusterId id);

    // StreamPermittedOverride
    NodeMutation setStreamPermittedOverride(NodeId node, StreamId evac,
                                            StreamId appr, bool permitted);
    NodeMutation eraseStreamPermittedOverride(NodeId node, StreamId evac,
                                              StreamId appr);

    // StreamIntergreenOverride
    NodeMutation setStreamIntergreenOverride(NodeId node, StreamId evac,
                                             StreamId appr, int t_maxEvAp,
                                             int t_maxApEv);
    NodeMutation eraseStreamIntergreenOverride(NodeId node, StreamId evac,
                                               StreamId appr);

    void solve() {
        for (NodeId id : dirty_) recomputeDerived(id);
        dirty_.clear();
    }

   private:
    GraphData data_;
    GraphDerived derived_;

    std::unordered_set<NodeId> dirty_;
    void recomputeDerived(NodeId id);

    SlotMap<NodeTag, Node>::VacatedSlot removeNodeImpl(NodeId id);
    void createNodeImpl(SlotMap<NodeTag, Node>::VacatedSlot vs);
    SlotMap<EntryTag, Entry>::VacatedSlot removeEntryImpl(EntryId id);
    void createEntryImpl(SlotMap<EntryTag, Entry>::VacatedSlot vs);
    SlotMap<ExitTag, Exit>::VacatedSlot removeExitImpl(ExitId id);
    void createExitImpl(SlotMap<ExitTag, Exit>::VacatedSlot vs);
    SlotMap<MovementTag, Movement>::VacatedSlot removeMovementImpl(
        MovementId id);
    void createMovementImpl(SlotMap<MovementTag, Movement>::VacatedSlot vs);
    SlotMap<EntryLaneTag, EntryLane>::VacatedSlot removeEntryLaneImpl(
        EntryLaneId id);
    void createEntryLaneImpl(SlotMap<EntryLaneTag, EntryLane>::VacatedSlot vs);
    SlotMap<ExitLaneTag, ExitLane>::VacatedSlot removeExitLaneImpl(
        ExitLaneId id);
    void createExitLaneImpl(SlotMap<ExitLaneTag, ExitLane>::VacatedSlot vs);
    SlotMap<CrosswalkTag, Crosswalk>::VacatedSlot removeCrosswalkImpl(
        CrosswalkId id);
    void createCrosswalkImpl(SlotMap<CrosswalkTag, Crosswalk>::VacatedSlot vs);
    SlotMap<CrosswalkSeriesTag, CrosswalkSeries>::VacatedSlot
    removeCrosswalkSeriesImpl(CrosswalkSeriesId id);
    void createCrosswalkSeriesImpl(
        SlotMap<CrosswalkSeriesTag, CrosswalkSeries>::VacatedSlot vs);
    SlotMap<ClusterTag, Cluster>::VacatedSlot removeClusterImpl(ClusterId id);
    void createClusterImpl(SlotMap<ClusterTag, Cluster>::VacatedSlot vs);
};