#pragma once
#include <expected>
#include <memory>
#include <optional>
#include <road_network/graph.hpp>

#include "command_stack.hpp"
#include "destroy_query.hpp"
#include "editor/control_editor.hpp"
#include "editor/demand_editor.hpp"
#include "editor/graph_editor.hpp"
#include "editor/signal_editor.hpp"
#include "road_network/util/ids.hpp"
#include "root/command/transaction.hpp"
#include "root/editor/errors/graph_errors.hpp"
#include "root/editor/errors/signal_errors.hpp"

class Document;
class NetworkEventBus;
class GraphUseCases;
class DemandUseCases;
class SignalUseCases;
class ControlUseCases;

class EditorContext {
   public:
    EditorContext(Document& doc, NetworkEventBus& bus);
    ~EditorContext();

    GraphUseCases& graph() { return *graphUseCases_; }
    DemandUseCases& demand() { return *demandUseCases_; }
    SignalUseCases& signal() { return *signalUseCases_; }
    ControlUseCases& control() { return *controlUseCases_; }
    CommandStack& commandStack() { return cmdStack_; }

    // Cross-domain Tx — used by GraphUseCases for multi-editor cascades
    std::expected<std::vector<MovementId>, MovementLayoutError> setEntryLayoutTx(
        Transaction& tx, EntryId id,
        std::vector<GraphEditor::MovementSpec> specs, std::vector<bool> shared);
    void removeNodeTx(Transaction& tx, NodeId id);
    void removeCrosswalkTx(Transaction& tx, CrosswalkId id);
    void removeCrosswalkSeriesTx(Transaction& tx, CrosswalkSeriesId id);
    void removeEntryTx(Transaction& tx, EntryId id);
    void removeExitTx(Transaction& tx, ExitId id);
    void setEntryLanesTx(Transaction& tx, EntryId id,
                         std::vector<EntryLaneSpec> specs);
    void setExitLanesTx(Transaction& tx, ExitId id,
                        std::vector<ExitLaneSpec> specs);
    void removeEntryLayoutTx(Transaction& tx, EntryId id);

    // Cross-domain Tx — graph parameter edits whose derived data feeds the
    // signal intergreen matrix and the control validation pass
    void moveEntryTx(Transaction& tx, EntryId id, Position position,
                     Vector2 heading);
    void moveExitTx(Transaction& tx, ExitId id, Position position,
                    Vector2 heading);
    void setEntrySpeedLimitTx(Transaction& tx, EntryId id, float speedLimit);
    void setExitSpeedLimitTx(Transaction& tx, ExitId id, float speedLimit);
    void setCrosswalkSeriesWalkSpeedTx(Transaction& tx, CrosswalkSeriesId id,
                                       float speed);
    CrosswalkId createCrosswalkTx(Transaction& tx, CrosswalkSeriesId parent,
                                  Line line);
    void setCrosswalkLineTx(Transaction& tx, CrosswalkId id, Line line);
    void setCrosswalkEdgeTx(Transaction& tx, CrosswalkId id,
                            Crosswalk::SegmentParams edge);
    void setCrosswalkSplitTx(Transaction& tx, CrosswalkId id,
                             Crosswalk::SegmentParams split);
    void setCrosswalkTypeTx(Transaction& tx, CrosswalkId id,
                            CrosswalkType type);
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

    // Cross-domain Tx — signal/control cascades, used by SignalUseCases
    std::expected<SignalGroupId, SignalGroupError> addMovementGroupTx(
        Transaction& tx, SignalConfigId config, std::vector<ClusterId> clusters,
        bool isProtected, int intervalMin);
    std::expected<SignalGroupId, SignalGroupError> addCrosswalkGroupTx(
        Transaction& tx, SignalConfigId config,
        std::vector<CrosswalkId> crosswalks, int intervalMin);
    std::expected<SignalGroupId, SignalGroupError> addTurnArrowGroupTx(
        Transaction& tx, SignalConfigId config, MovementId movement,
        int intervalMin);
    void setSignalGroupIntervalMinTx(Transaction& tx, SignalGroupId id,
                                     int min);
    void setSignalGroupProtectedTx(Transaction& tx, SignalGroupId id,
                                   bool isProtected);
    void removeSignalGroupTx(Transaction& tx, SignalGroupId id);
    void removeSignalConfigTx(Transaction& tx, SignalConfigId id);
    void removeIntersectionTx(Transaction& tx, IntersectionId id);

    struct OptimizerRunResult {
        SignalConfigId configId;
        TimingId timingId;
    };
    // Only single-node intersections are supported for now; returns nullopt
    // for multi-node ones.
    std::optional<OptimizerRunResult> runOptimizerTx(Transaction& tx,
                                                     IntersectionId at,
                                                     int cycleLength);

    // Read-only dry-run cascade query backing confirm dialogs.
    const DestroyQuery& destroyQuery() const { return destroyQuery_; }

   private:
    Document& doc_;
    CommandStack cmdStack_;
    GraphEditor graphEditor_;
    DemandEditor demandEditor_;
    SignalEditor signalEditor_;
    ControlEditor controlEditor_;
    DestroyQuery destroyQuery_;

    std::unique_ptr<GraphUseCases> graphUseCases_;
    std::unique_ptr<DemandUseCases> demandUseCases_;
    std::unique_ptr<SignalUseCases> signalUseCases_;
    std::unique_ptr<ControlUseCases> controlUseCases_;

    void removeEntryLayoutsTx(Transaction& tx, NodeId id);
    // graph-side change at the node invalidates the intergreen matrices and
    // timing validity of every config at the node's intersection
    void markSignalControlStaleAtNodeTx(Transaction& tx, NodeId id);
};
