#pragma once

#include <expected>
#include <road_network/graph.hpp>
#include <road_network/util/geometry/line.hpp>
#include <road_network/util/ids.hpp>

#include "errors/graph_errors.hpp"

class NetworkEventBus;
class Transaction;

class GraphEditor {
   public:
    GraphEditor(Graph& graph, NetworkEventBus& bus)
        : graph_(graph), bus_(bus) {}

    NodeId createNodeTx(Transaction& tx, Position center);
    EntryId createEntryTx(Transaction& tx, NodeId at, Position position,
                          Vector2 heading);
    ExitId createExitTx(Transaction& tx, NodeId at, Position position,
                        Vector2 heading);
    CrosswalkSeriesId createCrosswalkSeriesTx(Transaction& tx, NodeId at);
    CrosswalkId createCrosswalkTx(Transaction& tx, CrosswalkSeriesId parent,
                                  Line line);
    void setEntryLanesTx(Transaction& tx, EntryId at,
                         std::vector<EntryLaneSpec> specs);
    void setExitLanesTx(Transaction& tx, ExitId at,
                        std::vector<ExitLaneSpec> specs);

    void setMovementPathSpecTx(Transaction& tx, MovementId id,
                               MovementPathSpec spec);
    std::expected<void, MovementSplitError> setMovementSplitsTx(
        Transaction& tx, MovementId id, std::vector<int> splits);
    void setMovementTypeTx(Transaction& tx, MovementId id, MovementType type);
    void setCrosswalkSeriesPedPathTx(Transaction& tx, CrosswalkSeriesId id,
                                     Polyline path);
    void setCrosswalkTypeTx(Transaction& tx, CrosswalkId id,
                            CrosswalkType type);
    void setCrosswalkLineTx(Transaction& tx, CrosswalkId id, Line line);
    void setCrosswalkEdgeTx(Transaction& tx, CrosswalkId id,
                            Crosswalk::SegmentParams edge);
    void setCrosswalkSplitTx(Transaction& tx, CrosswalkId id,
                             Crosswalk::SegmentParams split);

    struct MovementSpec {
        ExitId exit;
        int laneCount;
    };
    std::vector<MovementId> setEntryLayoutTx(Transaction& tx, EntryId id,
                                             std::vector<MovementSpec> specs,
                                             std::vector<bool> shared);

    std::expected<void, MovementLayoutError> validateLayout(
        EntryId id, const std::vector<MovementSpec>& specs,
        const std::vector<bool>& shared);

    void removeCrosswalkSeriesTx(Transaction& tx, CrosswalkSeriesId id);
    void removeCrosswalkTx(Transaction& tx, CrosswalkId id);
    void removeEntryTx(Transaction& tx, EntryId id);
    void removeNodeTx(Transaction& tx, NodeId id);
    void removeExitTx(Transaction& tx, ExitId id);
    void clearEntryLayoutTx(Transaction& tx, EntryId id);

    void setStreamPermittedOverrideTx(Transaction& tx, NodeId node,
                                      StreamId evac, StreamId appr,
                                      bool permitted);
    void eraseStreamPermittedOverrideTx(Transaction& tx, NodeId node,
                                        StreamId evac, StreamId appr);

    void setStreamIntergreenOverrideTx(Transaction& tx, NodeId node,
                                       StreamId evac, StreamId appr,
                                       int t_maxEvAp, int t_maxApEv);
    void eraseStreamIntergreenOverrideTx(Transaction& tx, NodeId node,
                                         StreamId evac, StreamId appr);

   private:
    Graph& graph_;
    NetworkEventBus& bus_;

    void onNodeStale(Transaction& tx, NodeId id);
    MovementId createMovementTx(Transaction& tx, EntryId from, ExitId to,
                                std::vector<EntryLaneId> lanes,
                                std::vector<int> split);
    void removeMovementTx(Transaction& tx, MovementId id);
    ClusterId createClusterTx(Transaction& tx, EntryId at,
                              std::vector<MovementId> movements,
                              std::vector<EntryLaneId> lanes);
    void removeClusterTx(Transaction& tx, ClusterId id);
    std::expected<void, MovementSplitError> validateSplits(
        MovementId id, const std::vector<int>& splits);
};
