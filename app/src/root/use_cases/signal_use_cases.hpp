#pragma once

#include <expected>
#include <optional>
#include <road_network/util/ids.hpp>
#include <string>
#include <vector>

#include "root/editor/errors/signal_errors.hpp"
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

    std::expected<SignalGroupId, SignalGroupError> addMovementGroup(
        SignalConfigId config, std::vector<ClusterId> clusters,
        bool isProtected, int intervalMin);
    std::expected<SignalGroupId, SignalGroupError> addCrosswalkGroup(
        SignalConfigId config, std::vector<CrosswalkId> crosswalks,
        int intervalMin);
    std::expected<SignalGroupId, SignalGroupError> addTurnArrowGroup(
        SignalConfigId config, MovementId movement, int intervalMin);

    void setIntervalMin(SignalGroupId id, int min);
    void setProtected(SignalGroupId id, bool isProtected);
    void removeSignalGroup(SignalGroupId id);

    // right-turn movements eligible for a new turn-arrow group in this config
    [[nodiscard]] std::vector<MovementId> eligibleTurnArrowMovements(
        SignalConfigId config) const;

    // only single-node intersections are supported for now
    std::optional<EditorContext::OptimizerRunResult> runOptimizer(
        IntersectionId at, int cycleLength);

   private:
    SignalEditor& editor_;
    EditorContext& ec_;
    CommandStack& cmdStack_;
};
