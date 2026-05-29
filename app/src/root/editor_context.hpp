#pragma once
#include <memory>
#include <road_network/graph.hpp>

#include "command_stack.hpp"
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

class EditorContext {
   public:
    EditorContext(Document& doc, NetworkEventBus& bus);
    ~EditorContext();

    GraphUseCases& graph() { return *graphUseCases_; }
    DemandUseCases& demand() { return *demandUseCases_; }
    SignalUseCases& signal() { return *signalUseCases_; }
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

   private:
    Document& doc_;
    CommandStack cmdStack_;
    GraphEditor graphEditor_;
    DemandEditor demandEditor_;
    SignalEditor signalEditor_;

    std::unique_ptr<GraphUseCases> graphUseCases_;
    std::unique_ptr<DemandUseCases> demandUseCases_;
    std::unique_ptr<SignalUseCases> signalUseCases_;

    void removeEntryLayoutsTx(Transaction& tx, NodeId id);
};
