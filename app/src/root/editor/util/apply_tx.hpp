#pragma once
#include "bus/impl/event_bus.hpp"
#include "root/command/transaction.hpp"

template <class Mut, class E, class UndoE>
auto applyTx(Transaction& tx, Mut mut, EventBus<E>& bus, E event,
             EventBus<UndoE>& undoBus, UndoE undoEvent) {
    tx.recordUndo(std::move(mut.undo), undoBus, std::move(undoEvent));
    tx.recordEvent(bus, std::move(event));
    return mut.id;
}

template <class Mut, class E>
auto applyTxUpdate(Transaction& tx, Mut mut, EventBus<E>& bus, E event) {
    return applyTx(tx, std::move(mut), bus, event, bus, event);
}