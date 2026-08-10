#pragma once

#include "bus/dirty_event_bus.hpp"
#include "editor/control_editor.hpp"
#include "editor/demand_editor.hpp"
#include "editor/graph_editor.hpp"
#include "editor/signal_editor.hpp"

class Document;
class NetworkEventBus;

class Editor {
   public:
    Editor(Document& doc, NetworkEventBus& bus);

    GraphEditor& graph() { return ge_; }
    DemandEditor& demand() { return de_; }
    SignalEditor& signal() { return se_; }
    ControlEditor& control() { return ce_; }

    void registerSolves(Transaction& tx) {
        ge_.registerSolve(tx);
        de_.registerSolve(tx);
        se_.registerSolve(tx);
        ce_.registerSolve(tx);
    }

    std::optional<TimingId> runOptimizer(Transaction& tx, NodeId at,
                                         int cycleLength);

   private:
    DirtyEventBus dirty_;
    const Document& doc_;
    GraphEditor ge_;
    DemandEditor de_;
    SignalEditor se_;
    ControlEditor ce_;
};