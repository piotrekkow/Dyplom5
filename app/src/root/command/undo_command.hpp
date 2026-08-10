#pragma once

#include <functional>
#include <map>

#include "solve_hooks.hpp"
#include "solve_priority.hpp"

using UndoFn = std::move_only_function<void()>;
using EventFn = std::function<void()>;

struct UndoCommand {
    UndoCommand(const UndoCommand&) = delete;
    UndoCommand& operator=(const UndoCommand&) = delete;
    UndoCommand(UndoCommand&&) = default;
    UndoCommand& operator=(UndoCommand&&) = default;

    UndoCommand(std::vector<UndoFn> undos_,
                std::map<SolvePriority, SolveHooks> solves_,
                std::vector<EventFn> undoEvents_)
        : undos(std::move(undos_)),
          solves(std::move(solves_)),
          undoEvents(std::move(undoEvents_)) {}

    std::vector<UndoFn> undos;
    std::map<SolvePriority, SolveHooks> solves;
    std::vector<EventFn> undoEvents;
};