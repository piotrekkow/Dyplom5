#include "road_network/graph.hpp"

#include "derived/graph_compute.hpp"

NodeId Graph::parentOf(NodeId id) const { return id; }
NodeId Graph::parentOf(EntryId id) const {
    return data_.entries().get(id)->at();
}
NodeId Graph::parentOf(MovementId id) const {
    return parentOf(data_.movements().get(id)->from());
}
NodeId Graph::parentOf(ExitId id) const { return data_.exits().get(id)->at(); }
NodeId Graph::parentOf(CrosswalkId id) const {
    return parentOf(data_.crosswalks().get(id)->parent());
}
NodeId Graph::parentOf(CrosswalkSeriesId id) const {
    return data_.crosswalkSeries().get(id)->at();
}
NodeId Graph::parentOf(ClusterId id) const {
    return parentOf(data_.clusters().get(id)->at());
}

void Graph::link(NodeId id) {
    data_.streamIntergreenOverrides().ensure(id);
    data_.streamPermittedOverrides().ensure(id);
}
void Graph::link(EntryId id) {
    data_.nodes().get(parentOf(id))->entries().push_back(id);
}
void Graph::link(MovementId id) {
    auto* mov = data_.movements().get(id);
    auto* entry = data_.entries().get(mov->from());
    entry->movements().push_back(id);
}
void Graph::link(ExitId id) {
    data_.nodes().get(parentOf(id))->exits().push_back(id);
}
void Graph::link(CrosswalkId id) {
    auto* cw = data_.crosswalks().get(id);
    auto* cws = data_.crosswalkSeries().get(cw->parent());
    cws->crosswalks().push_back(id);
}
void Graph::link(CrosswalkSeriesId id) {
    data_.nodes().get(parentOf(id))->crosswalkSeries().push_back(id);
}
void Graph::link(ClusterId id) {
    auto* cls = data_.clusters().get(id);
    auto* entry = data_.entries().get(cls->at());
    entry->clusters().push_back(id);
    for (auto mid : cls->movements()) {
        auto* mov = data_.movements().get(mid);
        assert(!mov->cluster() && "movement already belongs to a cluster");
        mov->setCluster(id);
    }
}

void Graph::unlink(NodeId id) {
    auto* n = data_.nodes().get(id);
    assert(n->entries().empty() && "node still has entries");
    assert(n->exits().empty() && "node still has exits");
    assert(n->crosswalkSeries().empty() && "node still has cw series");
    data_.streamIntergreenOverrides().erase(id);
    data_.streamPermittedOverrides().erase(id);
}
void Graph::unlink(EntryId id) {
    auto* e = data_.entries().get(id);
    assert(e->movements().empty() && e->clusters().empty() &&
           "entry layout not empty");
    std::erase(data_.nodes().get(parentOf(id))->entries(), id);
}
void Graph::unlink(MovementId id) {
    auto* m = data_.movements().get(id);
    assert(!m->cluster() && "movement still referenced by a cluster");
    std::erase(data_.entries().get(m->from())->movements(), id);
}
void Graph::unlink(ExitId id) {
    std::erase(data_.nodes().get(parentOf(id))->exits(), id);
}
void Graph::unlink(CrosswalkId id) {
    auto* cw = data_.crosswalks().get(id);
    std::erase(data_.crosswalkSeries().get(cw->parent())->crosswalks(), id);
}
void Graph::unlink(CrosswalkSeriesId id) {
    auto* cs = data_.crosswalkSeries().get(id);
    assert(cs->crosswalks().empty() && "series still has crosswalks");
    std::erase(data_.nodes().get(parentOf(id))->crosswalkSeries(), id);
}
void Graph::unlink(ClusterId id) {
    auto* cl = data_.clusters().get(id);
    std::erase(data_.entries().get(cl->at())->clusters(), id);
    for (auto mid : cl->movements())
        data_.movements().get(mid)->setCluster(std::nullopt);
}

Mut<NodeId> Graph::createNode(Position center) {
    auto id = data_.nodes().emplace(center);
    return {.id = id, .undo = [this, id] {
                data_.nodes().release(detach(data_.nodes(), id));
            }};
}
Mut<NodeId> Graph::removeNode(NodeId id) {
    auto vs = detach(data_.nodes(), id);
    return {.id = id, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.nodes(), std::move(vs));
            }};
}

EMut<EntryId, NodeId> Graph::createEntry(NodeId at, Position position,
                                         Vector2 heading) {
    auto id = createChild(data_.entries(), at, position, heading);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.entries().release(detach(data_.entries(), id));
            }};
}
EMut<EntryId, NodeId> Graph::setParams(EntryId id, Entry::Params p) {
    return Base::setParams(data_.entries(), id, std::move(p));
}
EMut<EntryId, NodeId> Graph::removeEntry(EntryId id) {
    auto n = parentOf(id);
    auto vs = detach(data_.entries(), id);
    return {.id = id, .at = n, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.entries(), std::move(vs));
            }};
}

EMut<ExitId, NodeId> Graph::createExit(NodeId at, Position position,
                                       Vector2 heading) {
    auto id = createChild(data_.exits(), at, position, heading);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.exits().release(detach(data_.exits(), id));
            }};
}
EMut<ExitId, NodeId> Graph::setParams(ExitId id, Exit::Params p) {
    return Base::setParams(data_.exits(), id, std::move(p));
}
EMut<ExitId, NodeId> Graph::removeExit(ExitId id) {
    auto n = parentOf(id);
    auto vs = detach(data_.exits(), id);
    return {.id = id, .at = n, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.exits(), std::move(vs));
            }};
}

EMut<CrosswalkSeriesId, NodeId> Graph::createCrosswalkSeries(NodeId at) {
    auto id = createChild(data_.crosswalkSeries(), at);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.crosswalkSeries().release(
                    detach(data_.crosswalkSeries(), id));
            }};
}
EMut<CrosswalkSeriesId, NodeId> Graph::setParams(CrosswalkSeriesId id,
                                                 CrosswalkSeries::Params p) {
    return Base::setParams(data_.crosswalkSeries(), id, std::move(p));
}
EMut<CrosswalkSeriesId, NodeId> Graph::removeCrosswalkSeries(
    CrosswalkSeriesId id) {
    auto n = parentOf(id);
    auto vs = detach(data_.crosswalkSeries(), id);
    return {.id = id, .at = n, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.crosswalkSeries(), std::move(vs));
            }};
}

EMut<CrosswalkId, NodeId> Graph::createCrosswalk(CrosswalkSeriesId at,
                                                 Line anchor) {
    auto n = parentOf(at);
    auto id = createChild(data_.crosswalks(), at, anchor,
                          data_.crosswalkSeries().get(at)->params().type);
    return {.id = id, .at = n, .undo = [this, id] {
                data_.crosswalks().release(detach(data_.crosswalks(), id));
            }};
}
EMut<CrosswalkId, NodeId> Graph::setParams(CrosswalkId id,
                                           Crosswalk::Params p) {
    return Base::setParams(data_.crosswalks(), id, p);
}
EMut<CrosswalkId, NodeId> Graph::removeCrosswalk(CrosswalkId id) {
    auto n = parentOf(id);
    auto vs = detach(data_.crosswalks(), id);
    return {.id = id, .at = n, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.crosswalks(), std::move(vs));
            }};
}

EMut<MovementId, NodeId> Graph::createMovement(EntryId from, ExitId to,
                                               EntrySection lanes) {
    auto id = createChild(data_.movements(), from, to, lanes);
    return {.id = id, .at = parentOf(from), .undo = [this, id] {
                data_.movements().release(detach(data_.movements(), id));
            }};
}
EMut<MovementId, NodeId> Graph::setParams(MovementId id, Movement::Params p) {
    return Base::setParams(data_.movements(), id, std::move(p));
}
EMut<MovementId, NodeId> Graph::removeMovement(MovementId id) {
    auto n = parentOf(id);
    auto vs = detach(data_.movements(), id);
    return {.id = id, .at = n, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.movements(), std::move(vs));
            }};
}

EMut<ClusterId, NodeId> Graph::createCluster(EntryId at,
                                             std::vector<MovementId> movements,
                                             EntrySection lanes) {
    auto id = createChild(data_.clusters(), at, std::move(movements), lanes);
    return {.id = id, .at = parentOf(at), .undo = [this, id] {
                data_.clusters().release(detach(data_.clusters(), id));
            }};
}
EMut<ClusterId, NodeId> Graph::removeCluster(ClusterId id) {
    auto n = parentOf(id);
    auto vs = detach(data_.clusters(), id);
    return {.id = id, .at = n, .undo = [this, vs = std::move(vs)] mutable {
                attach(data_.clusters(), std::move(vs));
            }};
}

Mut<NodeId> Graph::setPermittedOverride(NodeId id, StreamId a, StreamId b,
                                        bool permitted) {
    assert(data_.nodes().alive(id));
    auto& arr = data_.streamPermittedOverrides();
    StreamMatrix<bool> m = arr.get(id) ? *arr.get(id) : StreamMatrix<bool>{};
    m.set(a, b, permitted, permitted);
    return setSlot(arr, id, std::move(m));
}
Mut<NodeId> Graph::clearPermittedOverride(NodeId id, StreamId a, StreamId b) {
    assert(data_.nodes().alive(id));
    auto& arr = data_.streamPermittedOverrides();
    StreamMatrix<bool> m = arr.get(id) ? *arr.get(id) : StreamMatrix<bool>{};
    m.reset(a, b);
    return setSlot(arr, id, std::move(m));
}

Mut<NodeId> Graph::setIntergreenOverride(NodeId id, StreamId evac,
                                         StreamId appr, int tMaxEvAp,
                                         int tMaxApEv) {
    assert(data_.nodes().alive(id));
    auto& arr = data_.streamIntergreenOverrides();
    StreamMatrix<int> m = arr.get(id) ? *arr.get(id) : StreamMatrix<int>{};
    m.set(evac, appr, tMaxEvAp, tMaxApEv);
    return setSlot(arr, id, std::move(m));
}
Mut<NodeId> Graph::clearIntergreenOverride(NodeId id, StreamId evac,
                                           StreamId appr) {
    assert(data_.nodes().alive(id));
    auto& arr = data_.streamIntergreenOverrides();
    StreamMatrix<int> m = arr.get(id) ? *arr.get(id) : StreamMatrix<int>{};
    m.reset(evac, appr);
    return setSlot(arr, id, std::move(m));
}

void Graph::recomputeDerived(NodeId id) {
    derived_.conflicts().set(id, graph_compute::conflictMap(data_, id));
    auto ig = graph_compute::streamIntergreenMaps(*this, id);
    derived_.streamIntergreenDiagnostics().set(id, std::move(ig.diag));
    if (const auto* ov = data_.streamIntergreenOverrides().get(id))
        ig.max = ig.max.combined(*ov);
    derived_.streamIntergreens().set(id, std::move(ig.max));
    auto spm = graph_compute::streamPermittedMap(*this, id);
    if (const auto* ov = data_.streamPermittedOverrides().get(id))
        spm = spm.combined(*ov);
    derived_.streamPermitted().set(id, std::move(spm));
    derived_.streamApproach().set(id, graph_compute::streamApproachMap(*this, id));
}
void Graph::evictDerived(NodeId id) {
    derived_.conflicts().erase(id);
    derived_.streamIntergreens().erase(id);
    derived_.streamIntergreenDiagnostics().erase(id);
    derived_.streamPermitted().erase(id);
    derived_.streamApproach().erase(id);
}