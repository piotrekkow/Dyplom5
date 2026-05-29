#pragma once

#include "command/undo_command.hpp"

class CommandStack {
   public:
    void push(UndoCommand uCmd) { undoStack_.push_back(std::move(uCmd)); }

    [[nodiscard]] bool canUndo() const { return !undoStack_.empty(); }

    void undo() {
        if (!canUndo()) return;
        auto cmd = std::move(undoStack_.back());
        undoStack_.pop_back();

        for (auto& u : cmd.undos) u();
        for (auto& [_, s] : cmd.solves) s();
        for (auto& e : cmd.undoEvents) e();
    }

   private:
    std::vector<UndoCommand> undoStack_;
};