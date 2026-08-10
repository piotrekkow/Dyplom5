#pragma once

#include "graph/graph_data.hpp"
#include "graph/graph_derived.hpp"
#include "util/mutation_base.hpp"

struct Regulations;
class Graph : public MutationBase<NodeId> {
    using Base = MutationBase<NodeId>;
    // Base reaches into link()/unlink()/parentOf() via self()
    friend Base;

   public:
    explicit Graph(const Regulations& regulations)
        : regulations_(regulations) {}
    [[nodiscard]] const GraphData& data() const { return data_; }
    [[nodiscard]] const GraphDerived& derived() const { return derived_; }
    [[nodiscard]] const Regulations& regulations() const {
        return regulations_;
    }

    struct Rederived {
        std::unordered_set<NodeId> nodes;
    };
    Rederived solve() {
        Rederived out;
        for (NodeId id : dirty<NodeId>()) {
            if (data_.nodes().alive(id)) {
                recomputeDerived(id);
                out.nodes.insert(id);
            } else
                evictDerived(id);
        }
        dirty<NodeId>().clear();
        return out;
    }

    // Node
    Mut<NodeId> createNode(Position center);
    Mut<NodeId> removeNode(NodeId id);

    // Entry
    EMut<EntryId, NodeId> createEntry(NodeId at, Position position,
                                      Vector2 heading);
    EMut<EntryId, NodeId> setParams(EntryId id, Entry::Params p);
    EMut<EntryId, NodeId> removeEntry(EntryId id);

    // Exit
    EMut<ExitId, NodeId> createExit(NodeId at, Position position,
                                    Vector2 heading);
    EMut<ExitId, NodeId> setParams(ExitId id, Exit::Params p);
    EMut<ExitId, NodeId> removeExit(ExitId id);

    // CrosswalkSeries
    EMut<CrosswalkSeriesId, NodeId> createCrosswalkSeries(NodeId at);
    EMut<CrosswalkSeriesId, NodeId> setParams(CrosswalkSeriesId id,
                                              CrosswalkSeries::Params p);
    EMut<CrosswalkSeriesId, NodeId> removeCrosswalkSeries(CrosswalkSeriesId id);

    // Crosswalk
    EMut<CrosswalkId, NodeId> createCrosswalk(CrosswalkSeriesId at,
                                              Line anchor);
    EMut<CrosswalkId, NodeId> setParams(CrosswalkId id, Crosswalk::Params p);
    EMut<CrosswalkId, NodeId> removeCrosswalk(CrosswalkId id);

    // Movement
    EMut<MovementId, NodeId> createMovement(EntryId from, ExitId to,
                                            EntrySection lanes);
    EMut<MovementId, NodeId> setParams(MovementId id, Movement::Params p);
    EMut<MovementId, NodeId> removeMovement(MovementId id);

    // Cluster
    EMut<ClusterId, NodeId> createCluster(EntryId at,
                                          std::vector<MovementId> movements,
                                          EntrySection lanes);
    EMut<ClusterId, NodeId> removeCluster(ClusterId id);

    // Overrides
    Mut<NodeId> setPermittedOverride(NodeId id, StreamId a, StreamId b,
                                     bool permitted);
    Mut<NodeId> clearPermittedOverride(NodeId id, StreamId a, StreamId b);

    Mut<NodeId> setIntergreenOverride(NodeId id, StreamId evac, StreamId appr,
                                      int tMaxEvAp, int tMaxApEv);
    Mut<NodeId> clearIntergreenOverride(NodeId id, StreamId evac,
                                        StreamId appr);

    // parentOf() - which bucket owns this entity (return type selects the
    // dirty set in the base)
    [[nodiscard]] NodeId parentOf(NodeId) const;
    [[nodiscard]] NodeId parentOf(EntryId) const;
    [[nodiscard]] NodeId parentOf(MovementId) const;
    [[nodiscard]] NodeId parentOf(ExitId) const;
    [[nodiscard]] NodeId parentOf(CrosswalkId) const;
    [[nodiscard]] NodeId parentOf(CrosswalkSeriesId) const;
    [[nodiscard]] NodeId parentOf(ClusterId) const;

   private:
    GraphData data_;
    GraphDerived derived_;
    const Regulations& regulations_;

    void recomputeDerived(NodeId id);
    void evictDerived(NodeId id);

    // link() - push into parent's child vector
    void link(NodeId);
    void link(EntryId);
    void link(MovementId);
    void link(ExitId);
    void link(CrosswalkId);
    void link(CrosswalkSeriesId);
    void link(ClusterId);

    // unlink() - erase from parent's child vector, erase derived mentions
    void unlink(NodeId);
    void unlink(EntryId);
    void unlink(MovementId);
    void unlink(ExitId);
    void unlink(CrosswalkId);
    void unlink(CrosswalkSeriesId);
    void unlink(ClusterId);
};