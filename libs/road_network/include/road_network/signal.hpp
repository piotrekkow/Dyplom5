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

    struct ConfigMutation {
        SignalConfigId id;
        NodeId at;
        std::move_only_function<void()> undo;
    };

    struct GroupMutation {
        SignalGroupId id;
        SignalConfigId config;
        NodeId at;
        std::move_only_function<void()> undo;
    };

    // config lifecycle
    ConfigMutation createConfig(NodeId at, std::string name);
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

    void solve(const GraphDerived& graphDerived) {
        for (SignalConfigId id : dirty_) recomputeDerived(graphDerived, id);
        dirty_.clear();
    }

   private:
    SignalData data_;
    SignalDerived derived_;

    std::unordered_set<SignalConfigId> dirty_;
    void recomputeDerived(const GraphDerived& graphDerived, SignalConfigId id);

    void createConfigImpl(SlotMap<SignalConfigTag, SignalConfig>::VacatedSlot vs);
    SlotMap<SignalConfigTag, SignalConfig>::VacatedSlot removeConfigImpl(
        SignalConfigId id);
    void createSignalGroupImpl(
        SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot vs);
    SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot removeSignalGroupImpl(
        SignalGroupId id);
};
