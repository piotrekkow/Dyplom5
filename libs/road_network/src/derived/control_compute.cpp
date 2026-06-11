#include "control_compute.hpp"

#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include "road_network/control/data/timing.hpp"
#include "road_network/graph/graph_data.hpp"
#include "road_network/signal/signal_data.hpp"
#include "road_network/signal/signal_derived.hpp"

namespace control_compute {

ConfigCompleteness configCompleteness(const GraphData& graph,
                                      const SignalData& signal,
                                      SignalConfigId id) {
    ConfigCompleteness out;
    const auto* cfg = signal.configs().get(id);
    if (!cfg) return out;
    const auto* intersection = signal.intersections().get(cfg->at());
    if (!intersection) return out;

    std::unordered_set<ClusterId> coveredClusters;
    std::unordered_set<CrosswalkId> coveredCrosswalks;
    for (SignalGroupId sgId : cfg->signalGroups()) {
        const auto* sg = signal.signalGroups().get(sgId);
        if (const auto* mg = std::get_if<MovementSignalGroup>(sg)) {
            coveredClusters.insert(mg->clusters().begin(),
                                   mg->clusters().end());
        } else if (const auto* cg = std::get_if<CrosswalkSignalGroup>(sg)) {
            coveredCrosswalks.insert(cg->crosswalks().begin(),
                                     cg->crosswalks().end());
        }
    }

    for (NodeId nId : intersection->nodes()) {
        const auto* node = graph.nodes().get(nId);
        if (!node) continue;
        for (EntryId eId : node->entries())
            for (ClusterId cId : graph.entries().get(eId)->clusters())
                if (!coveredClusters.contains(cId))
                    out.uncoveredClusters.push_back(cId);
        for (CrosswalkSeriesId csId : node->crosswalkSeries())
            for (CrosswalkId cwId :
                 graph.crosswalkSeries().get(csId)->crosswalks())
                if (!coveredCrosswalks.contains(cwId))
                    out.uncoveredCrosswalks.push_back(cwId);
    }
    return out;
}

TimingValidity timingValidity(const SignalData& signal,
                              const SignalDerived& signalDerived,
                              const Timing& timing) {
    TimingValidity v;
    const int C = timing.cycleLength();
    const auto* cfg = signal.configs().get(timing.at());
    if (!cfg || C <= 0) return v;

    // Green windows per group; AlwaysActive == green for the whole cycle.
    struct Window {
        int start;
        int dur;
    };
    std::unordered_map<SignalGroupId, std::vector<Window>> windows;
    for (SignalGroupId sgId : cfg->signalGroups()) {
        if (!timing.hasSequence(sgId)) continue;
        const Sequence& seq = timing.sequence(sgId);
        if (std::holds_alternative<AlwaysActive>(seq)) {
            windows[sgId].push_back({.start = 0, .dur = C});
            continue;
        }
        int min = std::visit([](const auto& g) { return g.intervalMin(); },
                             *signal.signalGroups().get(sgId));
        for (const Interval& iv :
             std::get<SignalSequence>(seq).intervals()) {
            if (iv.greenDuration() < min) v.intervalMinOk = false;
            int s = ((iv.greenStart() % C) + C) % C;
            windows[sgId].push_back({.start = s, .dur = iv.greenDuration()});
        }
    }

    // Intergreen: matrix entries exist only for conflicting group pairs.
    // Greens must not overlap and the approaching green may start no earlier
    // than t seconds after the evacuating green ended.
    const auto* matrix = signalDerived.intergreenMatrix().get(timing.at());
    if (!matrix) return v;

    auto ahead = [&](int a, int b) {  // forward cyclic distance a -> b
        int d = (b - a) % C;
        return d < 0 ? d + C : d;
    };
    for (const auto& [evac, row] : *matrix) {
        auto evIt = windows.find(evac);
        if (evIt == windows.end()) continue;
        for (const auto& [appr, t] : row) {
            auto apIt = windows.find(appr);
            if (appr == evac || apIt == windows.end()) continue;
            for (const Window& a : evIt->second) {
                for (const Window& b : apIt->second) {
                    if (a.dur >= C || b.dur >= C ||
                        ahead(a.start, b.start) < a.dur ||
                        ahead(b.start, a.start) < b.dur ||
                        ahead((a.start + a.dur) % C, b.start) < t) {
                        v.intergreenOk = false;
                    }
                }
            }
        }
    }
    return v;
}

}  // namespace control_compute
