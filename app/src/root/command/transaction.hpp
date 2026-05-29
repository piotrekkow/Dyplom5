#pragma once

#include <algorithm>
#include <cassert>
#include <map>
#include <ranges>

#include "bus/event_bus.hpp"
#include "solve_priority.hpp"
#include "undo_command.hpp"

class Transaction {
   public:
    Transaction() = default;

    ~Transaction() {
        if (!committed_) rollbackImpl();
    }

    Transaction(const Transaction&) = delete;
    Transaction& operator=(const Transaction&) = delete;
    Transaction(Transaction&&) = default;
    Transaction& operator=(Transaction&&) = default;

    template <typename E>
    void recordUndo(UndoFn undo, EventBus<E>& bus, E undoEvent) {
        assert(!committed_ && "record called after commit");
        undos_.emplace_back(std::move(undo));
        undoEvents_.emplace_back(
            [&bus, event = std::move(undoEvent)] { bus.publish(event); });
    }

    void recordUndoSilent(UndoFn undo) {
        assert(!committed_);
        undos_.emplace_back(std::move(undo));
    }

    template <typename E>
    void recordEvent(EventBus<E>& bus, E event) {
        assert(!committed_ && "record called after commit");
        events_.emplace_back(
            [&bus, event = std::move(event)] { bus.publish(event); });
    }

    template <typename E>
    void recordUndoEvent(EventBus<E>& bus, E event) {
        assert(!committed_ && "record called after commit");
        undoEvents_.emplace_back(
            [&bus, event = std::move(event)] { bus.publish(event); });
    }

    void addSolve(SolvePriority priority, std::function<void()> fn) {
        assert(!committed_ && "addSolve called after commit");
        solves_.try_emplace(priority, std::move(fn));
    }

    [[nodiscard]] UndoCommand commit() {
        assert(!committed_ && "double commit");

        committed_ = true;
        for (auto& [_, fn] : solves_) fn();
        for (auto& fn : events_) fn();
        events_.clear();

        std::ranges::reverse(undos_);
        std::ranges::reverse(undoEvents_);

        return {std::move(undos_), std::move(solves_), std::move(undoEvents_)};
    }

    void rollback() {
        assert(!committed_ && "rollback after commit");
        rollbackImpl();
        committed_ = true;  // sentinel: prevents destructor double-rollback
    }

    [[nodiscard]] bool committed() const { return committed_; }

   private:
    void rollbackImpl() {
        for (auto& undo : std::views::reverse(undos_)) {
            try {
                undo();
            } catch (...) {
                assert(false && "UndoFn threw — graph state may be corrupt");
            }
        }
        events_.clear();
        undos_.clear();
        undoEvents_.clear();
    }

    bool committed_ = false;
    std::vector<EventFn> events_;
    std::map<SolvePriority, std::function<void()>> solves_;

    std::vector<UndoFn> undos_;
    std::vector<EventFn> undoEvents_;
};