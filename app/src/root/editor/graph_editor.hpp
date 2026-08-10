#pragma once

#include <expected>
#include <geometry/line.hpp>
#include <geometry/polyline.hpp>
#include <geometry/position.hpp>
#include <regulations/types/stream_kind.hpp>
#include <road_network/graph/data/boundary/entry_lane_spec.hpp>
#include <road_network/graph/data/connector/movement_path_spec.hpp>
#include <road_network/graph/data/crosswalk/crosswalk_geometry.hpp>
#include <road_network/util/ids.hpp>
#include <unordered_set>

#include "util/editor_errors.hpp"

class Graph;
class NetworkEventBus;
class Transaction;
class DirtyEventBus;

class GraphEditor {
   public:
    GraphEditor(Graph& graph, NetworkEventBus& bus, DirtyEventBus& dirty)
        : graph_(graph), bus_(bus), dirty_(dirty) {}

    // creation
    NodeId createNode(Transaction& tx, Position center);
    EntryId createEntry(Transaction& tx, NodeId at, Position position,
                        Vector2 heading);
    ExitId createExit(Transaction& tx, NodeId at, Position position,
                      Vector2 heading);
    CrosswalkSeriesId createCrosswalkSeries(Transaction& tx, NodeId at);
    CrosswalkId createCrosswalk(Transaction& tx, CrosswalkSeriesId parent,
                                Line anchor);
    struct MovementSpec {
        ExitId exit;
        int laneCount;
    };
    std::expected<std::vector<ClusterId>, MovementLayoutError> setEntryLayout(
        Transaction& tx, EntryId id, std::vector<MovementSpec> specs,
        std::vector<bool> shared);

    // removal
    void removeNode(Transaction& tx, NodeId id);
    void removeEntry(Transaction& tx, EntryId id);
    void removeExit(Transaction& tx, ExitId id);
    void removeCrosswalk(Transaction& tx, CrosswalkId id);
    void removeCrosswalkSeries(Transaction& tx, CrosswalkSeriesId id);
    void clearEntryLayout(Transaction& tx, EntryId id);

    // override handling
    void setPermittedOverride(Transaction& tx, NodeId node, StreamId a,
                              StreamId b, bool permitted);
    void clearPermittedOverride(Transaction& tx, NodeId node, StreamId a,
                                StreamId b);
    void setIntergreenOverride(Transaction& tx, NodeId node, StreamId evac,
                               StreamId appr, int t_maxEvAp, int t_maxApEv);
    void clearIntergreenOverride(Transaction& tx, NodeId node, StreamId evac,
                                 StreamId appr);

    // setters
    void setEntryLanes(Transaction& tx, EntryId id,
                       std::vector<EntryLaneSpec> spec);
    void setExitLanes(Transaction& tx, ExitId id,
                      std::vector<double> spec);  // spec - lane widths

    // position + heading move as one gesture, mirroring create
    void moveEntry(Transaction& tx, EntryId id, Position position,
                   Vector2 heading);
    void moveExit(Transaction& tx, ExitId id, Position position,
                  Vector2 heading);

    void setEntrySpeedLimit(Transaction& tx, EntryId id, double speedLimit);
    void setExitSpeedLimit(Transaction& tx, ExitId id, double speedLimit);

    void setMovementPathSpec(Transaction& tx, MovementId id,
                             MovementPathSpec pathSpec);
    void setMovementType(Transaction& tx, MovementId id, MovementType type);
    void setMovementSplitPerLane(Transaction& tx, MovementId id,
                                 std::vector<int> splitPerLane);

    void setCSeriesCriticalPath(Transaction& tx, CrosswalkSeriesId id,
                                Polyline path);
    void setCSeriesCriticalSpeed(Transaction& tx, CrosswalkSeriesId id,
                                 double speed);
    void setCSeriesType(Transaction& tx, CrosswalkSeriesId id,
                        CrosswalkType type);

    void setCrosswalkGeometry(Transaction& tx, CrosswalkId id,
                              CrosswalkGeometryInput geometry);

    void registerSolve(Transaction& tx);
    void rederive();
    void flush();

   private:
    Graph& graph_;
    NetworkEventBus& bus_;
    DirtyEventBus& dirty_;

    std::unordered_set<NodeId> reNodes_;

    MovementId createMovement(Transaction& tx, EntryId from, ExitId to,
                              int firstIdx, int laneCount);
    ClusterId createCluster(Transaction& tx, EntryId at,
                            std::vector<MovementId> movements, int firstIdx,
                            int laneCount);
    void removeMovement(Transaction& tx, MovementId id);
    void removeCluster(Transaction& tx, ClusterId id);

    std::expected<void, MovementLayoutError> validateLayout(
        EntryId id, const std::vector<MovementSpec>& specs,
        const std::vector<bool>& shared);

    std::vector<EntryId> entriesFeedingExit(ExitId exit);
};