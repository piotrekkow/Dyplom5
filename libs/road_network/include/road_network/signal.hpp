#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "signal/signal_data.hpp"
#include "signal/signal_derived.hpp"
#include "util/ids.hpp"
#include <functional>

class GraphDerived;

class Signal {
   public:
    const SignalData& data() const { return data_; }
    const SignalDerived& derived() const { return derived_; }

    struct IntersectionMutation {
        IntersectionId id;
        std::move_only_function<void()> undo;
    };

    struct ConfigMutation {
        SignalConfigId id;
        IntersectionId at;
        std::move_only_function<void()> undo;
    };

    struct GroupMutation {
        SignalGroupId id;
        SignalConfigId config;
        IntersectionId at;
        std::move_only_function<void()> undo;
    };

    // intersection lifecycle
    IntersectionMutation createIntersection(std::string name);
    // pre: intersection has no configs and no nodes (caller removes/detaches
    // them first)
    IntersectionMutation removeIntersection(IntersectionId id);
    // pre: node is not part of any intersection
    IntersectionMutation addNode(IntersectionId id, NodeId node);
    // pre: node is part of this intersection
    IntersectionMutation removeNode(IntersectionId id, NodeId node);

    // config lifecycle
    ConfigMutation createConfig(IntersectionId at, std::string name);
    // pre: config has no signal groups (caller removes them first)
    ConfigMutation removeConfig(SignalConfigId id);
    ConfigMutation renameConfig(SignalConfigId id, std::string name);

    // signal group creation — specific type constructed in editor
    GroupMutation createSignalGroup(SignalGroup sg);
    GroupMutation createMovementGroup(SignalConfigId config,
                                      std::vector<ClusterId> clusters,
                                      std::vector<MovementId> movements,
                                      bool isProtected, int intervalMin);
    GroupMutation createCrosswalkGroup(SignalConfigId config,
                                       std::vector<CrosswalkId> crosswalks,
                                       int intervalMin);
    GroupMutation createTurnArrowGroup(SignalConfigId config,
                                       MovementId movement, int intervalMin);

    // signal group mutations
    GroupMutation setIntervalMin(SignalGroupId id, int min);
    GroupMutation setProtected(SignalGroupId id, bool isProtected);
    GroupMutation removeSignalGroup(SignalGroupId id);

    // External invalidation: graph-side parameter changes (stream intergreen /
    // permitted maps, speeds, geometry) feed the config's intergreen matrix.
    void markStale(SignalConfigId id) { dirty_.insert(id); }

    void solve(const GraphDerived& graphDerived) {
        for (SignalConfigId id : dirty_) recomputeDerived(graphDerived, id);
        dirty_.clear();
    }

   private:
    SignalData data_;
    SignalDerived derived_;

    std::unordered_set<SignalConfigId> dirty_;
    void recomputeDerived(const GraphDerived& graphDerived, SignalConfigId id);

    void addNodeImpl(IntersectionId id, NodeId node);
    void removeNodeImpl(IntersectionId id, NodeId node);
    void createConfigImpl(SlotMap<SignalConfigTag, SignalConfig>::VacatedSlot vs);
    SlotMap<SignalConfigTag, SignalConfig>::VacatedSlot removeConfigImpl(
        SignalConfigId id);
    void createSignalGroupImpl(
        SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot vs);
    SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot removeSignalGroupImpl(
        SignalGroupId id);
};
