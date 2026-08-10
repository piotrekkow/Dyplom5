#include "road_network/util/compute_helpers.hpp"

#include <regulations/regulations.hpp>

#include "road_network/demand/demand_derived.hpp"
#include "road_network/graph/graph_data.hpp"
#include "road_network/signal.hpp"

namespace compute {
double classifyAngle(const GraphData& graph, EntryId entry, ExitId exit) {
    const auto* e = graph.entries().get(entry);
    const auto* x = graph.exits().get(exit);
    assert(e && x);
    auto ev = e->params().heading;
    auto mv = x->params().position - e->params().position;
    auto xv = x->params().heading;
    return ev.angleTo(mv) + mv.angleTo(xv);
}
TurnKind turnKind(const GraphData& graph, const Regulations& regulations,
                  MovementId id) {
    const auto* m = graph.movements().get(id);
    assert(m);

    return regulations.classifyTurn(classifyAngle(graph, m->from(), m->to()));
}

std::unordered_map<SignalGroupId, double> groupMaxFlowRatios(
    const Signal& signal, const DemandDerived& demand, NodeId at) {
    const auto* gptr = signal.data().nodeGroups().get(at);
    assert(gptr);

    const auto& groups = *gptr;
    std::unordered_map<SignalGroupId, double> maxFlow;
    for (auto gid : groups) {
        const auto* g = signal.data().signalGroups().get(gid);
        auto msg = std::get_if<MovementSignalGroup>(g);
        // crosswalk groups carry no flow but still need an entry
        maxFlow[gid] = 0.0;
        if (!msg) continue;
        bool isProtected =
            signal.derived().protectionResolutions().get(gid)->isProtected();

        double best = 0.0;
        for (auto mid : msg->movements()) {
            auto fr = demand.flowRatios().get(mid);
            double val = 0.0;
            if (fr)
                val = isProtected ? fr->prot.value_or(0.0)
                                  : fr->perm.value_or(0.0);
            if (val > best) best = val;
        }
        maxFlow[gid] = best;
    }
    return maxFlow;
}

std::unordered_map<SignalGroupId, control::GroupParams> groupParams(
    const Signal& signal, const Regulations& regulations, NodeId at) {
    const auto* gptr = signal.data().nodeGroups().get(at);
    assert(gptr);

    std::unordered_map<SignalGroupId, control::GroupParams> params;
    for (auto gid : *gptr) {
        const auto* g = signal.data().signalGroups().get(gid);
        auto [kind, intervalMinFixed] = std::visit(
            [](const auto& sg) {
                return std::pair{sg.params().kind,
                                 sg.params().intervalMinFixed};
            },
            *g);

        const auto aspect = regulations.aspects(kind);
        params[gid] = control::GroupParams{
            .minActiveTime = std::max(intervalMinFixed, aspect.minGreen.fixed),
            .from = {.aspect = aspect.toActive.aspect,
                     .duration = aspect.toActive.duration,
                     .isIntergreen = aspect.toActive.isIntergreen},
            .to = {.aspect = aspect.toInactive.aspect,
                   .duration = aspect.toInactive.duration,
                   .isIntergreen = aspect.toInactive.isIntergreen},
        };
    }
    return params;
}

int resolveStreamIntergreen(const StreamMatrix<int>& intergreens,
                            const StreamMatrix<bool>& permitted, StreamId evac,
                            StreamId appr, bool evacProtected,
                            bool apprProtected) {
    const int* ig = intergreens.find(evac, appr);
    if (!ig) return -1;

    if (!evacProtected && !apprProtected) {
        const bool* fwd = permitted.find(evac, appr);
        const bool* rev = permitted.find(appr, evac);
        if (fwd && *fwd && rev && *rev) return -1;
    }
    return *ig;
}
}  // namespace compute