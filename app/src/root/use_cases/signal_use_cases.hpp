#pragma once

#include <optional>
#include <road_network/util/ids.hpp>
#include <string>
#include <vector>

#include "root/editor_context.hpp"

class SignalEditor;
class CommandStack;

class SignalUseCases {
   public:
    SignalUseCases(SignalEditor& editor, EditorContext& ec,
                   CommandStack& cmdStack);

    SignalConfigId createConfig(NodeId at, std::string name);
    void renameConfig(SignalConfigId id, std::string name);
    void removeConfig(SignalConfigId id);

    SignalGroupId addMovementGroup(SignalConfigId config,
                                   std::vector<ClusterId> clusters,
                                   bool isProtected, int intervalMin);
    SignalGroupId addCrosswalkGroup(SignalConfigId config,
                                    std::vector<CrosswalkId> crosswalks,
                                    int intervalMin);
    SignalGroupId addTurnArrowGroup(SignalConfigId config, MovementId movement,
                                    int intervalMin);

    void setIntervalMin(SignalGroupId id, int min);
    void setProtected(SignalGroupId id, bool isProtected);
    void removeSignalGroup(SignalGroupId id);

    std::optional<EditorContext::OptimizerRunResult> runOptimizer(
        NodeId at, int cycleLength);

   private:
    SignalEditor& editor_;
    EditorContext& ec_;
    CommandStack& cmdStack_;
};
