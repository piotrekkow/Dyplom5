#pragma once

#include <algorithm>
#include <cassert>
#include <map>
#include <ranges>

#include "bus/impl/event_bus.hpp"
#include "solve_hooks.hpp"
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

    template <typename E>
    void recordEvent(EventBus<E>& bus, E event) {
        assert(!committed_ && "record called after commit");
        events_.emplace_back(
            [&bus, event = std::move(event)] { bus.publish(event); });
    }

    void addSolve(SolvePriority priority, SolveHooks solve) {
        assert(!committed_ && "addSolve called after commit");
        solves_.try_emplace(priority, std::move(solve));
    }

    void solveUpto(SolvePriority upto) {
        for (auto& [prio, solve] : solves_) {
            if (prio >= upto) break;
            solve.rederive();
        }
    }

    [[nodiscard]] UndoCommand commit() {
        assert(!committed_ && "double commit");

        committed_ = true;
        for (auto& [_, s] : solves_) s.rederive();
        for (auto& [_, s] : solves_) s.flush();
        for (auto& fn : events_) fn();
        events_.clear();

        std::ranges::reverse(undos_);
        std::ranges::reverse(undoEvents_);

        return {std::move(undos_), std::move(solves_), std::move(undoEvents_)};
    }

    void rollback() {
        assert(!committed_ && "rollback after commit");
        rollbackImpl();
        committed_ = true;  // prevents destructor double-rollback
    }

   private:
    void rollbackImpl() {
        for (auto& undo : std::views::reverse(undos_)) {
            try {
                undo();
            } catch (...) {
                assert(false && "UndoFn threw: graph state may be corrupt");
            }
        }
        for (auto& [_, s] : solves_) s.rederive();
        events_.clear();
        undos_.clear();
        undoEvents_.clear();
    }

    bool committed_ = false;
    std::vector<EventFn> events_;

    std::map<SolvePriority, SolveHooks> solves_;

    std::vector<UndoFn> undos_;
    std::vector<EventFn> undoEvents_;
};