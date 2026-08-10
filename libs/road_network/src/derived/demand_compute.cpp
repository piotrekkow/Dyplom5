#include "demand_compute.hpp"

#include <cassert>
#include <cmath>
#include <functional>
#include <regulations/regulations.hpp>

#include "road_network/demand.hpp"
#include "road_network/demand/demand_data.hpp"
#include "road_network/demand/derived/flow_ratio.hpp"
#include "road_network/demand/derived/saturation.hpp"
#include "road_network/graph/data/connector/cluster.hpp"
#include "road_network/graph/derived/graph_geometry.hpp"
#include "road_network/graph/graph_data.hpp"
#include "road_network/util/compute_helpers.hpp"

namespace {
std::optional<EntryId> findOpposingEntry(const GraphData& graph,
                                         const Regulations& reg, EntryId from) {
    const auto* e = graph.entries().get(from);
    assert(e);

    Vector2 fromHeading = e->params().heading;
    const auto* node = graph.nodes().get(e->at());
    std::optional<EntryId> result = std::nullopt;

    for (auto otherId : node->entries()) {
        if (otherId == from) continue;
        const auto* other = graph.entries().get(otherId);
        Vector2 otherHeading = other->params().heading;

        if (reg.isOpposing(fromHeading, otherHeading)) result = otherId;
    }

    return result;
}
}  // namespace

namespace demand_compute {
std::vector<Saturation> entryLaneSaturations(const GraphData& graph,
                                             const DemandData& demand,
                                             const Regulations& regulations,
                                             EntryId id) {
    auto entry = graph.entries().get(id);
    assert(entry);
    if (!entry) {
        return {};
    }

    // idx -> movements at that lane idx
    std::vector<std::vector<MovementId>> laneMovements(
        entry->params().lanes.size());

    // movements guaranteed to be ordered leftmost->rightmost in
    // entry::movements()
    for (auto mId : entry->movements()) {
        const auto& sec = graph.movements().get(mId)->lanes();
        for (int i = sec.firstIdx; i <= sec.lastIdx(); ++i)
            laneMovements[i].push_back(mId);
    }

    struct MovMeta {
        double Qm;
        Saturation Sm;
        int pm = 0;
        int qm = 0;
        int size() { return pm + qm; }
    };

    std::unordered_map<MovementId, MovMeta> meta;

    for (auto mId : entry->movements()) {
        auto* flow = demand.flows().get(mId);
        meta[mId].Qm = flow ? flow->eqVehiclesPerHour() : 0.0;

        const auto* overrides = demand.saturationOverrides().get(mId);
        auto base = baseSaturation(graph, demand, regulations, mId);
        meta[mId].Sm =
            overrides ? Saturation{.prot = overrides->prot.value_or(base.prot),
                                   .perm = overrides->perm.value_or(base.perm)}
                      : base;
    }

    for (const auto& lane : laneMovements) {
        bool shared = lane.size() > 1;
        for (auto mId : lane) {
            shared ? ++meta[mId].qm : ++meta[mId].pm;
            assert(meta[mId].qm <= 2);  // movements only have two edges
        }
    }

    using LoadFactorPair = Saturation;
    using FlowPair = Saturation;
    std::unordered_map<const Cluster*, LoadFactorPair> laneGroups;
    // int -> lane idx
    using FlowMap =
        std::unordered_map<int, std::unordered_map<MovementId, FlowPair>>;

    // 1. Compute initial Qmj values
    // Exclusive: Qmj = Qm / (pm + 0.5*qm)
    // Shared:    Qmj = 0.5*Qm / (pm + 0.5*qm)
    auto initializeFlows = [&](FlowMap& qmj, const Cluster* cl) {
        const auto& sec = cl->lanes();
        for (int i = sec.firstIdx; i <= sec.lastIdx(); ++i) {
            bool isShared = laneMovements[i].size() > 1;
            for (auto m : laneMovements[i]) {
                auto& mm = meta[m];
                double coeff = isShared ? 0.5 : 1.0;
                double Qmj = coeff * mm.Qm / (mm.pm + 0.5 * mm.qm);
                qmj[i][m] = {.prot = Qmj, .perm = Qmj};
            }
        }
    };

    // 2. Calculate load factor Y for lane group
    // $$Y = \frac{1}{n}\sum_j \sum_m \frac{Q_{mj}}{S_m}$$
    auto computeLoadFactorY = [&](const FlowMap& qmj, const Cluster* cl) {
        LoadFactorPair sum = {.prot = 0.0, .perm = 0.0};
        const auto& sec = cl->lanes();
        for (int i = sec.firstIdx; i <= sec.lastIdx(); ++i) {
            for (auto [m, q] : qmj.at(i)) {
                sum += q / meta[m].Sm;
            }
        }
        assert(cl->lanes().laneCount > 0);
        laneGroups[cl] = (1.0 / cl->lanes().laneCount) * sum;
    };

    // 3. Recaluclate Qmj values
    auto redistributeFlows = [&](FlowMap& qmj, const Cluster* cl) -> void {
        FlowMap qmjSnapshot = qmj;
        for (auto mId : cl->movements()) {
            auto& mm = meta[mId];
            const auto& y_lg = laneGroups[cl];
            if (mm.size() == 1 || mm.qm == 0)
                continue;  // initial Qmj already correct
            auto& sec = graph.movements().get(mId)->lanes();
            for (int i = sec.firstIdx; i <= sec.lastIdx(); ++i) {
                // Exclusive lanes: $$Q_{mj}^{(ex)} = Y \cdot S_m$$
                if (laneMovements[i].size() == 1) qmj[i][mId] = y_lg * mm.Sm;

                // Shared lane — movement spanning most lanes (e.g. T in `T T
                // TR`):
                // $$Q_{mj} = Q_m - \sum_{ex} Q_{mj}^{(ex)}$$
                if (laneMovements[i].size() > 1 && mm.qm == 1) {
                    FlowPair Qm = {.prot = mm.Qm, .perm = mm.Qm};
                    qmj[i][mId] = Qm - mm.pm * y_lg * mm.Sm;
                }

                // Shared lane — when two shared lanes present (e.g. T in `LT
                // T TR`):
                // $$Q_{mj} = S_m \cdot \left(Y -
                // \frac{Q_{mj,other}}{S_{m,other}}\right)$$
                if (laneMovements[i].size() > 1 && mm.qm == 2) {
                    LoadFactorPair yOther = {.prot = 0.0, .perm = 0.0};
                    for (auto other : laneMovements[i]) {
                        if (other == mId) continue;
                        yOther += qmjSnapshot[i][other] / meta[other].Sm;
                    }
                    LoadFactorPair dY = y_lg - yOther;
                    qmj[i][mId].prot =
                        dY.prot > 0.0 ? mm.Sm.prot * dY.prot : 0.0;
                    qmj[i][mId].perm =
                        dY.perm > 0.0 ? mm.Sm.perm * dY.perm : 0.0;
                }
            }
        }
    };

    auto movQtotal = [&](FlowMap& qmj, MovementId mId) -> FlowPair {
        FlowPair total{.prot = 0.0, .perm = 0.0};
        const auto& sec = graph.movements().get(mId)->lanes();
        for (int i = sec.firstIdx; i <= sec.lastIdx(); ++i)
            total += qmj[i][mId];

        return total;
    };

    // Normalize per-lane flows so they sum to Qm — divergence can accumulate
    // from the iterative recalc.
    auto ensureNoLostFlow = [&](FlowMap& qmj, const Cluster* cl) {
        for (auto mId : cl->movements()) {
            if (meta[mId].qm == 0) continue;
            auto coeff = meta[mId].Qm / movQtotal(qmj, mId);
            const auto& sec = graph.movements().get(mId)->lanes();
            for (int i = sec.firstIdx; i <= sec.lastIdx(); ++i)
                qmj[i][mId] = qmj[i][mId] * coeff;
        }
    };

    auto harmonicSat = [&](FlowMap& qmj, int lane, auto field) -> double {
        double Qjtotal = 0.0;
        for (auto mId : laneMovements[lane])
            Qjtotal += std::invoke(field, qmj[lane][mId]);
        if (Qjtotal <= 1e-9) return 0.0;
        double inv_S = 0.0;
        for (auto mId : laneMovements[lane]) {
            double um = std::invoke(field, qmj[lane][mId]) / Qjtotal;
            inv_S += um / std::invoke(field, meta[mId].Sm);
        }
        return std::max(0.0, 1.0 / inv_S);
    };

    FlowMap qmj;
    std::unordered_set<const Cluster*> unresolved_y_lg;

    // initial pass
    // Steps 1 and 2
    for (auto& cid : entry->clusters()) {
        const auto* cl = graph.clusters().get(cid);
        initializeFlows(qmj, cl);
        computeLoadFactorY(qmj, cl);
        if (cl->movements().size() > 1) {
            unresolved_y_lg.insert(cl);
        }
    }

    // Repeat steps 2-3 until dY < 0.005
    for (int iter = 0; iter < 100; ++iter) {
        if (unresolved_y_lg.empty()) break;
        std::vector<const Cluster*> converged;
        for (auto cl : unresolved_y_lg) {
            redistributeFlows(qmj, cl);
            LoadFactorPair yOld = laneGroups[cl];
            computeLoadFactorY(qmj, cl);
            LoadFactorPair delta = laneGroups[cl] - yOld;
            if (std::abs(delta.prot) < 0.005 && std::abs(delta.perm) < 0.005)
                converged.push_back(cl);
        }
        for (auto cl : converged) {
            ensureNoLostFlow(qmj, cl);
            unresolved_y_lg.erase(cl);
        }
    }

    for (auto lg : unresolved_y_lg) {
        ensureNoLostFlow(qmj, lg);
        // TODO: SaturationDiagnostic
    }

    // Calculate movement share at lane $j$: $$u_m = \frac{Q_{mj}}{\sum_M
    // Q_{mj}}$$ and harmonic-mean adjusted saturation flow: $$S_j =
    // \left(\sum_m \frac{u_m}{S_m}\right)^{-1}$$
    std::vector<Saturation> Sj(entry->params().lanes.size());
    for (auto cl : entry->clusters()) {
        const auto& sec = graph.clusters().get(cl)->lanes();
        for (int i = sec.firstIdx; i <= sec.lastIdx(); ++i) {
            Sj[i].prot = harmonicSat(qmj, i, &Saturation::prot);
            Sj[i].perm = harmonicSat(qmj, i, &Saturation::perm);
        }
    }
    return Sj;
}

FlowRatio flowRatio(const GraphData& graph, const Demand& demand,
                    MovementId id) {
    const auto* mov = graph.movements().get(id);
    const auto* saturations =
        demand.derived().laneSaturations().get(mov->from());
    const auto* flow = demand.data().flows().get(id);
    if (!saturations || !flow) return {};

    const auto& lanes = mov->lanes();
    Saturation movSat{.prot = 0, .perm = 0};
    for (int i = lanes.firstIdx; i <= lanes.lastIdx(); ++i) {
        movSat += saturations->at(i);
    }

    // A zero adjusted saturation (the all-flows-zero degenerate state, or a
    // fully blocked permitted regime) has no meaningful ratio: leave the side
    // absent rather than dividing to inf/NaN.
    const double q = flow->eqVehiclesPerHour();
    FlowRatio out;
    if (movSat.prot > 0.0) out.prot = q / movSat.prot;
    if (movSat.perm > 0.0) out.perm = q / movSat.perm;
    return out;
}

Saturation baseSaturation(const GraphData& graph, const DemandData& demand,
                          const Regulations& regulations, MovementId id) {
    auto entry = graph.movements().get(id)->from();
    auto oppId = findOpposingEntry(graph, regulations, entry);
    double Qn = 0;
    if (oppId) {
        const auto* opp = graph.entries().get(*oppId);
        size_t laneCount = opp->params().lanes.size();
        for (auto mId : opp->movements()) {
            auto flow = demand.flows().get(mId);
            double eqFlow = flow ? flow->eqVehiclesPerHour() : 0.0;

            auto turn = compute::turnKind(graph, regulations, mId);
            Qn += eqFlow * regulations.opposingFlowShare(turn, laneCount);
        }
    }

    auto turn = compute::turnKind(graph, regulations, id);
    return {.prot = regulations.baseSaturationProt(turn),
            .perm = regulations.baseSaturationPerm(turn, Qn)};
}
}  // namespace demand_compute
