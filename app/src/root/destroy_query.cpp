#include "destroy_query.hpp"

#include <road_network/control.hpp>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>

#include "document.hpp"
#include "editor/signal_editor.hpp"

DestroyQuery::DestroyQuery(const Document& doc, const SignalEditor& signal)
    : doc_(doc), signal_(signal) {}

void DestroyQuery::accEntryLayout(Acc& acc, EntryId id) const {
    const auto* entry = doc_.graph().data().entries().get(id);
    NodeId at = entry->at();
    for (MovementId m : entry->movements()) acc.movements.insert(m);
    for (SignalGroupId sg : signal_.groupsReferencing(at, entry->clusters(),
                                                      entry->movements()))
        acc.groups.insert(sg);
}

void DestroyQuery::accNodeLayouts(Acc& acc, NodeId id) const {
    const auto* node = doc_.graph().data().nodes().get(id);
    for (EntryId eId : node->entries()) accEntryLayout(acc, eId);
}

void DestroyQuery::accCrosswalk(Acc& acc, CrosswalkId id) const {
    const auto* cw = doc_.graph().data().crosswalks().get(id);
    NodeId at = doc_.graph().data().crosswalkSeries().get(cw->parent())->at();
    for (SignalGroupId sg : signal_.groupsReferencingCrosswalk(at, id))
        acc.groups.insert(sg);
}

void DestroyQuery::accConfig(Acc& acc, SignalConfigId id) const {
    acc.configs.insert(id);
    const auto* cfg = doc_.signal().data().configs().get(id);
    for (SignalGroupId sg : cfg->signalGroups()) acc.groups.insert(sg);
    if (const auto* cc = doc_.control().data().configControls().get(id))
        for (TimingId t : cc->timings) acc.timings.insert(t);
}

void DestroyQuery::accIntersection(Acc& acc, IntersectionId id) const {
    acc.intersections.insert(id);
    for (SignalConfigId cfg : signal_.configsOf(id)) accConfig(acc, cfg);
}

DestroyQuery::Impact DestroyQuery::toImpact(const Acc& acc) const {
    Impact impact;
    // count only movements carrying non-default demand data
    for (MovementId m : acc.movements) {
        const auto* flow = doc_.demand().data().flows().get(m);
        const auto* sat = doc_.demand().data().saturationOverrides().get(m);
        bool hasFlow = flow && flow->eqVehiclesPerHour() > 0.f;
        bool hasSat = sat && (sat->prot || sat->perm);
        if (hasFlow || hasSat) ++impact.flows;
    }
    impact.signalGroups = static_cast<int>(acc.groups.size());
    impact.timings = static_cast<int>(acc.timings.size());
    impact.configs = static_cast<int>(acc.configs.size());
    impact.intersections = static_cast<int>(acc.intersections.size());
    return impact;
}

DestroyQuery::Impact DestroyQuery::removeNode(NodeId id) const {
    Acc acc;
    if (auto intersection = signal_.intersectionAt(id)) {
        const auto* in =
            doc_.signal().data().intersections().get(*intersection);
        if (in->nodes().size() == 1) {
            accIntersection(acc, *intersection);
        } else {
            const auto* node = doc_.graph().data().nodes().get(id);
            for (CrosswalkSeriesId csId : node->crosswalkSeries())
                for (CrosswalkId cwId : doc_.graph()
                                            .data()
                                            .crosswalkSeries()
                                            .get(csId)
                                            ->crosswalks())
                    accCrosswalk(acc, cwId);
        }
    }
    accNodeLayouts(acc, id);
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::removeEntry(EntryId id) const {
    Acc acc;
    accNodeLayouts(acc, doc_.graph().data().entries().get(id)->at());
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::removeExit(ExitId id) const {
    Acc acc;
    accNodeLayouts(acc, doc_.graph().data().exits().get(id)->at());
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::setEntryLanes(EntryId id) const {
    Acc acc;
    accEntryLayout(acc, id);
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::setExitLanes(ExitId id) const {
    Acc acc;
    accNodeLayouts(acc, doc_.graph().data().exits().get(id)->at());
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::setEntryLayout(EntryId id) const {
    Acc acc;
    accEntryLayout(acc, id);
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::removeCrosswalk(CrosswalkId id) const {
    Acc acc;
    accCrosswalk(acc, id);
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::removeCrosswalkSeries(
    CrosswalkSeriesId id) const {
    Acc acc;
    for (CrosswalkId cwId :
         doc_.graph().data().crosswalkSeries().get(id)->crosswalks())
        accCrosswalk(acc, cwId);
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::removeSignalGroup(SignalGroupId id) const {
    Acc acc;
    acc.groups.insert(id);
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::removeSignalConfig(SignalConfigId id) const {
    Acc acc;
    accConfig(acc, id);
    return toImpact(acc);
}

DestroyQuery::Impact DestroyQuery::removeIntersection(IntersectionId id) const {
    Acc acc;
    accIntersection(acc, id);
    return toImpact(acc);
}
