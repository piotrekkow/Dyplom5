#pragma once

#include <expected>
#include <road_network/control/data/stage_step.hpp>
#include <string>
#include <unordered_set>

#include "bus/impl/dirty_events.hpp"
#include "bus/subscription_bag.hpp"
#include "road_network/util/ids.hpp"
#include "util/editor_errors.hpp"

class Control;
class Demand;
class NetworkEventBus;
class Transaction;
class Signal;
class Graph;
class DirtyEventBus;
class Regulations;
class NodeControl;

class ControlEditor {
   public:
    ControlEditor(Control& control, const Signal& signal, const Demand& demand,
                  const Graph& graph, NetworkEventBus& bus,
                  DirtyEventBus& dirty, const Regulations& regulations);

    TimingId createTiming(Transaction& tx, NodeId at, std::string name);
    void setTimingName(Transaction& tx, TimingId id, std::string name);
    std::expected<void, TimingErrors> setTimingSteps(
        Transaction& tx, TimingId id, std::vector<StageStep> steps);
    void removeTiming(Transaction& tx, TimingId id);

    std::expected<StageId, StageErrors> createStage(
        Transaction& tx, NodeId at, std::unordered_set<SignalGroupId> active);
    std::expected<void, StageErrors> setStageGroups(
        Transaction& tx, StageId id, std::unordered_set<SignalGroupId> active);
    void removeStage(Transaction& tx, StageId id);

    StageTransitionId createTransition(Transaction& tx, NodeId at, StageId from,
                                       StageId to);
    void removeTransition(Transaction& tx, StageTransitionId id);

    void registerSolve(Transaction& tx);
    void rederive();
    void flush();

   private:
    Control& control_;
    const Signal& signal_;
    const Demand& demand_;
    const Graph& graph_;  // node alive checks in solve() and flush()
    NetworkEventBus& bus_;
    DirtyEventBus& dirty_;
    const Regulations& regulations_;
    SubscriptionBag subs_;
    std::unordered_set<NodeId> reNodes_;

    void onNodeUpdated(UpdatedTx<NodeId>);
    void onNodeRemoved(RemovedTx<NodeId>);
    void onSignalGroupCreated(CreatedTx<SignalGroupId, NodeId>);
    void onSignalGroupRemoved(RemovedTx<SignalGroupId, NodeId>);
    void clearNodeControl(Transaction& tx, const NodeControl* nc);
    void clearTimingsWhereSteps(Transaction& tx, NodeId at, auto pred);
    std::expected<control::Stage<SignalGroupId>, StageErrors> stageBody(
        std::unordered_set<SignalGroupId> active, NodeId at);
};