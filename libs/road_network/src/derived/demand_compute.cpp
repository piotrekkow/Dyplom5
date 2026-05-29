#include "demand_compute.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>

#include "road_network/demand/demand_data.hpp"
#include "road_network/graph/derived/graph_geometry.hpp"
#include "road_network/graph/graph_data.hpp"

namespace demand_compute {
namespace {
Saturation movementBaseSatTable(TurnType turn, float Qn) {
    switch (turn) {
        default:
        case TurnType::THROUGH:
            return {.prot = 1900.f, .perm = 1900.f};

        case TurnType::LEFT:
            return {.prot = 1750.f, .perm = 1620.f - 1.75f * Qn};

        case TurnType::RIGHT:
            return {.prot = 1600.f, .perm = 800.f};

        case TurnType::UTURN:
            return {.prot = 1470.f, .perm = 1360.f - 2.f * Qn};
    }
}

float computeQn(const GraphData& graph, const DemandData& demand, EntryId id) {
    auto oppId = graph_geometry::findOpposingEntry(graph, id);
    if (!oppId) return 0.f;

    const auto* opp = graph.entries().get(*oppId);
    float Qn = 0.f;
    for (auto mId : opp->movements()) {
        Flow flow =
            demand.flows().get(mId) ? *demand.flows().get(mId) : Flow{0.f, 0.f};
        const auto turn = graph_geometry::classifyTurn(graph, mId);

        switch (turn) {
            case TurnType::THROUGH:
                Qn += flow.eqVehiclesPerHour();
                break;
            case TurnType::RIGHT:
                if (opp->lanes().size() == 1)
                    Qn += flow.eqVehiclesPerHour();
                else if (opp->lanes().size() == 2)
                    Qn += flow.eqVehiclesPerHour() * 0.5f;
                // 3+ lanes: right turn does not contribute
                break;
            default:
                break;
        }
    }
    return Qn;
}

struct MovMeta {
    float Qm;
    Saturation Sm;
    int pm = 0;
    int qm = 0;
    int size() { return pm + qm; }
};

Saturation movementSaturation(const GraphData& graph, const DemandData& demand,
                              float Qn, MovementId mId) {
    auto turn = graph_geometry::classifyTurn(graph, mId);
    auto baseSat = movementBaseSatTable(turn, Qn);
    const auto* satOverrides = demand.saturationOverrides().get(mId);
    return satOverrides ? Saturation{satOverrides->prot.value_or(baseSat.prot),
                                     satOverrides->perm.value_or(baseSat.perm)}
                        : baseSat;
}

}  // namespace

std::unordered_map<EntryLaneId, Saturation> entryLaneSaturation(
    const GraphData& graph, const DemandData& demand, EntryId id) {
    auto entry = graph.entries().get(id);
    // TODO: SaturationDiagnostic
    assert(entry && "Satuartion - Invalid entry id");
    if (!entry) {
        return {};
    }

    // 0.1 use only movements with lanes for computation, ensure valid order
    std::vector<MovementId> entryMovements;
    float last = std::numeric_limits<float>::max();
    for (auto mId : entry->movements()) {
        if (graph.movements().get(mId)->lanes().empty()) continue;
        auto angle = graph_geometry::classifyAngle(graph, mId);
        if (angle > last) {
            // TODO: SaturationDiagnostic
            assert("Saturation - Invalid movement lane order");
            return {};
        }
        last = angle;
        entryMovements.emplace_back(mId);
    }

    // 0.2 establish helper map - lanes to movements
    std::unordered_map<EntryLaneId, std::vector<MovementId>> laneMovements;
    for (auto& mId : entryMovements) {
        for (auto& lId : graph.movements().get(mId)->lanes())
            laneMovements[lId].push_back(mId);
    }

    // 0.3 establish derivable movement metadata
    std::unordered_map<MovementId, MovMeta> meta;
    float Qn = computeQn(graph, demand, id);

    for (auto mId : entryMovements) {
        auto* flow = demand.flows().get(mId);
        meta[mId].Qm = flow ? flow->eqVehiclesPerHour() : 0.f;
        meta[mId].Sm = movementSaturation(graph, demand, Qn, mId);
    }

    for (auto& [laneId, mIds] : laneMovements) {
        bool shared = mIds.size() > 1;
        for (auto mId : mIds) {
            shared ? ++meta[mId].qm : ++meta[mId].pm;
            assert(meta[mId].qm <= 2);  // movements only have two edges
        }
    }

    // 0.3 establish lane groups
    using LoadFactorPair = Saturation;
    struct LaneGroup {
        std::unordered_set<MovementId> movements;
        std::unordered_set<EntryLaneId> entryLanes;
        LoadFactorPair y = {0.f, 0.f};
    };
    std::vector<LaneGroup> laneGroups;

    for (const auto& [laneId, mIds] : laneMovements) {
        assert(!mIds.empty());
        bool shared = false;
        if (!laneGroups.empty()) {
            for (auto mId : mIds) {
                if (laneGroups.back().movements.contains(mId)) {
                    shared = true;
                    break;
                }
            }
        }
        if (!shared) {
            laneGroups.push_back({});
        }
        for (auto mId : mIds) {
            laneGroups.back().movements.insert(mId);
            laneGroups.back().entryLanes.insert(laneId);
        }
    }

    using FlowPair = Saturation;
    using FlowMap =
        std::unordered_map<EntryLaneId,
                           std::unordered_map<MovementId, FlowPair>>;

    // 1. Compute initial Qmj values
    // Exclusive: Qmj = Qm / (pm + 0.5*qm)
    // Shared:    Qmj = 0.5*Qm / (pm + 0.5*qm)
    auto initializeFlows = [&](FlowMap& qmj, const LaneGroup* lg) {
        for (auto lane : lg->entryLanes) {
            bool isShared = laneMovements[lane].size() > 1;
            for (auto m : laneMovements[lane]) {
                auto& mm = meta[m];
                float coeff = isShared ? 0.5f : 1.0f;
                float Qmj = coeff * mm.Qm /
                            (static_cast<float>(mm.pm) +
                             0.5f * static_cast<float>(mm.qm));
                qmj[lane][m] = {Qmj, Qmj};
            }
        }
    };

    // 2. Calculate load factor Y for lane group
    // $$Y = \frac{1}{n}\sum_j \sum_m \frac{Q_{mj}}{S_m}$$
    auto computeLoadFactorY = [&](const FlowMap& qmj, LaneGroup* lg) {
        LoadFactorPair sum = {0.f, 0.f};
        assert(!lg->entryLanes.empty());
        for (auto lane : lg->entryLanes) {
            for (auto [m, q] : qmj.at(lane)) {
                sum += q / meta[m].Sm;
            }
        }
        lg->y = (1.f / static_cast<float>(lg->entryLanes.size())) * sum;
    };

    // 3. Recaluclate Qmj values
    auto redistributeFlows = [&](FlowMap& qmj, const LaneGroup* lg) -> void {
        FlowMap qmjSnapshot = qmj;
        for (auto mId : lg->movements) {
            auto& mm = meta[mId];
            if (mm.size() == 1 || mm.qm == 0)
                continue;  // initial Qmj already correct
            auto& lanes = graph.movements().get(mId)->lanes();
            for (auto lId : lanes) {
                // Exclusive lanes: $$Q_{mj}^{(ex)} = Y \cdot S_m$$
                if (laneMovements[lId].size() == 1)
                    qmj[lId][mId] = lg->y * mm.Sm;

                // Shared lane — movement spanning most lanes (e.g. T in `T T
                // TR`):
                // $$Q_{mj} = Q_m - \sum_{ex} Q_{mj}^{(ex)}$$
                if (laneMovements[lId].size() > 1 && mm.qm == 1) {
                    FlowPair Qm = {mm.Qm, mm.Qm};
                    qmj[lId][mId] =
                        Qm - static_cast<float>(mm.pm) * lg->y * mm.Sm;
                }

                // Shared lane — when two shared lanes present (e.g. T in `LT
                // T TR`):
                // $$Q_{mj} = S_m \cdot \left(Y -
                // \frac{Q_{mj,other}}{S_{m,other}}\right)$$
                if (laneMovements[lId].size() > 1 && mm.qm == 2) {
                    LoadFactorPair yOther = {0.f, 0.f};
                    for (auto other : laneMovements[lId]) {
                        if (other == mId) continue;
                        yOther += qmjSnapshot[lId][other] / meta[other].Sm;
                    }
                    LoadFactorPair dY = lg->y - yOther;
                    qmj[lId][mId].prot =
                        dY.prot > 0.f ? mm.Sm.prot * dY.prot : 0.f;
                    qmj[lId][mId].perm =
                        dY.perm > 0.f ? mm.Sm.perm * dY.perm : 0.f;
                }
            }
        }
    };

    auto movQtotal = [&](FlowMap& qmj, MovementId mId) -> FlowPair {
        FlowPair total{0.f, 0.f};
        for (auto& lId : graph.movements().get(mId)->lanes()) {
            total += qmj[lId][mId];
        }
        return total;
    };

    // Normalize per-lane flows so they sum to Qm — divergence can accumulate
    // from the iterative recalc.
    auto ensureNoLostFlow = [&](FlowMap& qmj, const LaneGroup* lg) {
        for (auto mId : lg->movements) {
            if (meta[mId].qm == 0) continue;
            auto coeff = meta[mId].Qm / movQtotal(qmj, mId);
            for (auto& lId : graph.movements().get(mId)->lanes()) {
                qmj[lId][mId] = qmj[lId][mId] * coeff;
            }
        }
    };

    auto harmonicSat = [&](FlowMap& qmj, auto lane, auto field) -> float {
        float Qjtotal = 0.f;
        for (auto mId : laneMovements[lane])
            Qjtotal += std::invoke(field, qmj[lane][mId]);
        if (Qjtotal <= 1e-6f) return 0.f;
        float inv_S = 0.f;
        for (auto mId : laneMovements[lane]) {
            float um = std::invoke(field, qmj[lane][mId]) / Qjtotal;
            inv_S += um / std::invoke(field, meta[mId].Sm);
        }
        return std::max(0.f, 1.f / inv_S);
    };

    FlowMap qmj;
    std::unordered_set<LaneGroup*> unresolvedY_lg;

    // initial pass
    // Steps 1 and 2
    for (auto& lg : laneGroups) {
        initializeFlows(qmj, &lg);
        computeLoadFactorY(qmj, &lg);
        if (lg.movements.size() > 1) {
            unresolvedY_lg.insert(&lg);
        }
    }

    // Repeat steps 2-3 until dY < 0.005
    for (int iter = 0; iter < 100; ++iter) {
        if (unresolvedY_lg.empty()) break;
        std::vector<LaneGroup*> converged;
        for (auto lg : unresolvedY_lg) {
            redistributeFlows(qmj, lg);
            LoadFactorPair yOld = lg->y;
            computeLoadFactorY(qmj, lg);
            LoadFactorPair delta = lg->y - yOld;
            if (std::abs(delta.prot) < 0.005f && std::abs(delta.perm) < 0.005f)
                converged.push_back(lg);
        }
        for (auto lg : converged) {
            ensureNoLostFlow(qmj, lg);
            unresolvedY_lg.erase(lg);
        }
    }

    for (auto lg : unresolvedY_lg) {
        ensureNoLostFlow(qmj, lg);
        // TODO: SaturationDiagnostic
        // std::cerr << "Qmj calculation did not converge for movements: ";
        // for (auto m : lg->movements) {
        //     std::cerr << "(" << m.index() << ") ";
        // }
        // std::cerr << '\n';
    }

    // Calculate movement share at lane $j$: $$u_m = \frac{Q_{mj}}{\sum_M
    // Q_{mj}}$$ and harmonic-mean adjusted saturation flow: $$S_j =
    // \left(\sum_m \frac{u_m}{S_m}\right)^{-1}$$
    std::unordered_map<EntryLaneId, Saturation> Sj;
    for (auto& lg : laneGroups) {
        for (auto lane : lg.entryLanes) {
            Sj[lane].prot = harmonicSat(qmj, lane, &Saturation::prot);
            Sj[lane].perm = harmonicSat(qmj, lane, &Saturation::perm);
        }
    }
    return Sj;
}
}  // namespace demand_compute
