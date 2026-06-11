#pragma once

#include <optional>
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

    // Intersection Tx
    // pre: every node is alive and not part of another intersection
    IntersectionId createIntersectionTx(Transaction& tx,
                                        const std::vector<NodeId>& nodes,
                                        std::string name);
    void addNodeToIntersectionTx(Transaction& tx, IntersectionId id,
                                 NodeId node);
    void removeNodeFromIntersectionTx(Transaction& tx, IntersectionId id,
                                      NodeId node);
    // Tx half of a cross-domain cascade — EditorContext removes the
    // intersection's configs first; detaches remaining nodes itself
    void removeIntersectionTx(Transaction& tx, IntersectionId id);

    // Config Tx
    SignalConfigId createConfigTx(Transaction& tx, IntersectionId at,
                                  std::string name);
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

    // const queries for EditorContext cascade decisions; node-keyed queries
    // resolve through the node's owning intersection
    [[nodiscard]] std::optional<IntersectionId> intersectionAt(NodeId id) const;
    [[nodiscard]] std::vector<SignalConfigId> configsOf(IntersectionId id) const;
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
