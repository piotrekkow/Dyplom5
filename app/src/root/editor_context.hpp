#pragma once
#include <memory>
#include <optional>
#include <road_network/graph.hpp>

#include "command_stack.hpp"
#include "editor/control_editor.hpp"
#include "editor/demand_editor.hpp"
#include "editor/graph_editor.hpp"
#include "editor/signal_editor.hpp"
#include "road_network/util/ids.hpp"
#include "root/command/transaction.hpp"
#include "root/editor/errors/graph_errors.hpp"

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

    // Cross-domain Tx — signal/control cascades, used by SignalUseCases
    SignalGroupId addMovementGroupTx(Transaction& tx, SignalConfigId config,
                                     std::vector<ClusterId> clusters,
                                     bool isProtected, int intervalMin);
    SignalGroupId addCrosswalkGroupTx(Transaction& tx, SignalConfigId config,
                                      std::vector<CrosswalkId> crosswalks,
                                      int intervalMin);
    SignalGroupId addTurnArrowGroupTx(Transaction& tx, SignalConfigId config,
                                      MovementId movement, int intervalMin);
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

   private:
    Document& doc_;
    CommandStack cmdStack_;
    GraphEditor graphEditor_;
    DemandEditor demandEditor_;
    SignalEditor signalEditor_;
    ControlEditor controlEditor_;

    std::unique_ptr<GraphUseCases> graphUseCases_;
    std::unique_ptr<DemandUseCases> demandUseCases_;
    std::unique_ptr<SignalUseCases> signalUseCases_;
    std::unique_ptr<ControlUseCases> controlUseCases_;

    void removeEntryLayoutsTx(Transaction& tx, NodeId id);
};
