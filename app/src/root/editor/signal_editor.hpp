#pragma once

#include <road_network/signal.hpp>
#include <road_network/util/ids.hpp>
#include <string>
#include <vector>

class Graph;
class NetworkEventBus;
class Transaction;

class SignalEditor {
   public:
    SignalEditor(Signal& signal, const Graph& graph, NetworkEventBus& bus);

    // Config Tx
    SignalConfigId createConfigTx(Transaction& tx, NodeId at, std::string name);
    void renameConfigTx(Transaction& tx, SignalConfigId id, std::string name);

    // Signal group Tx
    SignalGroupId addMovementGroupTx(Transaction& tx, SignalConfigId config,
                                     std::vector<ClusterId> clusters,
                                     bool isProtected, int intervalMin);
    SignalGroupId addCrosswalkGroupTx(Transaction& tx, SignalConfigId config,
                                      std::vector<CrosswalkId> crosswalks,
                                      int intervalMin);
    SignalGroupId addTurnArrowGroupTx(Transaction& tx, SignalConfigId config,
                                      MovementId movement, int intervalMin);
    void setIntervalMinTx(Transaction& tx, SignalGroupId id, int min);
    void setProtectedTx(Transaction& tx, SignalGroupId id, bool isProtected);

    // Tx halves of cross-domain cascades — called only by EditorContext,
    // which cleans the control domain first
    void removeSignalGroupTx(Transaction& tx, SignalGroupId id);
    void removeConfigTx(Transaction& tx, SignalConfigId id);

    // const queries for EditorContext cascade decisions
    [[nodiscard]] std::vector<SignalConfigId> configsAt(NodeId id) const;
    [[nodiscard]] std::vector<SignalGroupId> groupsReferencing(
        NodeId at, const std::vector<ClusterId>& clusters,
        const std::vector<MovementId>& movements) const;
    [[nodiscard]] std::vector<SignalGroupId> groupsReferencingCrosswalk(
        NodeId at, CrosswalkId id) const;

   private:
    Signal& signal_;
    const Graph& graph_;
    NetworkEventBus& bus_;

    SignalGroupId addSignalGroupTx(Transaction& tx, Signal::GroupMutation mut);
};
