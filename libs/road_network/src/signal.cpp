#include "road_network/signal.hpp"

#include "derived/signal_compute.hpp"
#include "road_network/graph.hpp"
#include "road_network/signal/data/crosswalk_signal_group.hpp"
#include "road_network/signal/data/movement_signal_group.hpp"

[[nodiscard]] NodeId Signal::parentOf(SignalGroupId id) const {
    auto sg = data_.signalGroups().get(id);
    return std::visit([](const auto& x) { return x.node(); }, *sg);
}
void Signal::link(SignalGroupId id) {
    const auto node = parentOf(id);
    data_.nodeGroups().ensure(node).push_back(id);
}
void Signal::unlink(SignalGroupId id) {
    assert(data_.signalGroups().alive(id));
    const auto node = parentOf(id);
    auto ng = data_.nodeGroups().get(node);
    assert(ng);
    std::erase(*ng, id);
    if (ng->empty()) data_.nodeGroups().erase(node);
}

EMut<SignalGroupId, NodeId> Signal::createSignalGroup(SignalGroup sg) {
    auto id = createChild(data_.signalGroups(), std::move(sg));
    auto at = parentOf(id);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.signalGroups().release(detach(data_.signalGroups(), id));
            }};
}

template <typename Group>
EMut<SignalGroupId, NodeId> Signal::setGroupParams(SignalGroupId id,
                                                   typename Group::Params p) {
    assert(data_.signalGroups().alive(id));
    auto at = parentOf(id);
    auto& g = std::get<Group>(*data_.signalGroups().get(id));
    typename Group::Params old = g.params();
    g.setParams(p);
    markDirty(at);
    return {.id = id, .at = at, .undo = [this, id, at, old] {
                std::get<Group>(*data_.signalGroups().get(id)).setParams(old);
                markDirty(at);
            }};
}

EMut<SignalGroupId, NodeId> Signal::createMovementGroup(
    NodeId node, EntryId entry, std::vector<ClusterId> clusters,
    std::vector<MovementId> movements, int minIntervalFixed) {
    return createSignalGroup(
        MovementSignalGroup{node, entry, std::move(clusters),
                            std::move(movements), minIntervalFixed});
}
EMut<SignalGroupId, NodeId> Signal::createCrosswalkGroup(
    NodeId node, std::vector<CrosswalkId> crosswalks, SignalKind kind,
    int minIntervalFixed) {
    return createSignalGroup(CrosswalkSignalGroup{node, std::move(crosswalks),
                                                  kind, minIntervalFixed});
}

EMut<SignalGroupId, NodeId> Signal::setParams(SignalGroupId id,
                                              MovementSignalGroup::Params p) {
    return setGroupParams<MovementSignalGroup>(id, p);
}
EMut<SignalGroupId, NodeId> Signal::setParams(SignalGroupId id,
                                              CrosswalkSignalGroup::Params p) {
    return setGroupParams<CrosswalkSignalGroup>(id, p);
}

EMut<SignalGroupId, NodeId> Signal::removeSignalGroup(SignalGroupId id) {
    auto at = parentOf(id);
    auto vs = detach(data_.signalGroups(), id);
    return {.id = id, .at = at, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.signalGroups(), std::move(vs));
            }};
}

Signal::Rederived Signal::solve(const Graph& graph) {
    Rederived out;
    for (NodeId id : dirty<NodeId>()) {
        if (graph.data().nodes().alive(id)) {
            recomputeDerived(graph, id);
            out.nodes.insert(id);
        } else
            evictDerived(id);
    }
    dirty<NodeId>().clear();
    return out;
}

void Signal::recomputeDerived(const Graph& graph, NodeId id) {
    // set protection first
    auto prot = signal_compute::protectionResolution(data_, graph.data(),
                                                     regulations_, id);
    for (const auto& [gid, val] : prot)
        derived_.protectionResolutions().set(gid, val);
    // only then calculate matrix
    auto ig = signal_compute::intergreenMatrix(graph.derived(), *this,
                                               regulations_, id);
    // approach caps need the intergreen matrix to know the permitted pairs
    auto ap = signal_compute::approachMatrix(graph.derived(), *this, id, ig);
    derived_.intergreenMatrices().set(id, std::move(ig));
    derived_.approachMatrices().set(id, std::move(ap));
}

void Signal::evictDerived(NodeId id) {
    derived_.intergreenMatrices().erase(id);
    derived_.approachMatrices().erase(id);
}