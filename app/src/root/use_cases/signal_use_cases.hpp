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

    // pre: every node is alive and not part of another intersection
    IntersectionId createIntersection(const std::vector<NodeId>& nodes,
                                      std::string name);
    void removeIntersection(IntersectionId id);
    void addNodeToIntersection(IntersectionId id, NodeId node);
    void removeNodeFromIntersection(IntersectionId id, NodeId node);

    SignalConfigId createConfig(IntersectionId at, std::string name);
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

    // only single-node intersections are supported for now
    std::optional<EditorContext::OptimizerRunResult> runOptimizer(
        IntersectionId at, int cycleLength);

   private:
    SignalEditor& editor_;
    EditorContext& ec_;
    CommandStack& cmdStack_;
};
