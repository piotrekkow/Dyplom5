#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <future>
#include <limits>
#include <mutex>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <signal_optimizer/optimizer.hpp>
#include <unordered_map>

#include "config_generator.hpp"
#include "delay.hpp"
#include "green_time.hpp"
#include "phase_cover.hpp"
#include "phase_generator.hpp"
#include "phase_sequence.hpp"
#include "road_network/signal/data/sequence.hpp"
#include "types.hpp"

namespace {
// Build the optimizer's input from domain objects for a specific node
GInput buildInput(NodeId nodeId, const Graph& graph, const Demand& demand,
                  int T) {
    const auto& gdata = graph.data();
    const auto& gderived = graph.derived();

    GInput inp{.entries = {}, .series = {}, .M = {}, .P = {}, .T = T};

    if (const auto* m = gderived.streamIntergreen().get(nodeId)) inp.M = *m;
    if (const auto* p = gderived.streamPermitted().get(nodeId)) inp.P = *p;
    const auto* node = gdata.nodes().get(nodeId);
    if (!node) return inp;

    //
    for (EntryId eid : node->entries()) {
        const auto* entry = gdata.entries().get(eid);
        if (!entry || entry->clusters().empty()) continue;

        GInput::EntryData edata{
            .id = eid, .heading = entry->heading(), .clusters = {}};

        for (ClusterId cid : entry->clusters()) {
            const auto* cluster = gdata.clusters().get(cid);
            if (!cluster) continue;
            GInput::ClusterData cdata{
                .id = cid,
                .movements = {},
                .lanes = {},
                .saturation = {},
                .Q = 0,
            };

            std::unordered_map<EntryLaneId, GInput::EntryLaneData> laneMap;
            for (EntryLaneId lid : cluster->lanes()) {
                const auto* sat = demand.derived().laneSaturations().get(lid);
                assert(sat);
                GInput::EntryLaneData ldata{
                    .id = lid,
                    .saturation = *sat,
                };

                laneMap.emplace(lid, ldata);
                cdata.saturation += ldata.saturation;
                cdata.lanes.push_back(ldata);
            }

            for (MovementId mid : cluster->movements()) {
                const auto* movement = gdata.movements().get(mid);
                if (!movement) continue;

                GInput::MovementData mdata{
                    .id = mid,
                    .lanes = {},
                    .turn = *gderived.turnTypes().get(mid)};

                for (auto lid : movement->lanes()) {
                    auto it = laneMap.find(lid);
                    if (it != laneMap.end()) mdata.lanes.push_back(it->second);
                }
                const auto* Q = demand.data().flows().get(mid);
                assert(Q);
                cdata.Q += Q->eqVehiclesPerHour();
                cdata.movements.push_back(std::move(mdata));
            }
            edata.clusters.push_back(std::move(cdata));
        }
        inp.entries.push_back(std::move(edata));
    }

    //
    for (CrosswalkSeriesId csid : node->crosswalkSeries()) {
        const auto* cs = gdata.crosswalkSeries().get(csid);
        if (!cs || cs->crosswalks().empty()) continue;

        int minGreen =
            std::max(4, static_cast<int>(std::ceil(cs->length() /
                                                   cs->pedestrianWalkSpeed())));
        GInput::SeriesData sdata{
            .id = csid, .crosswalks = cs->crosswalks(), .minGreen = minGreen};
        inp.series.push_back(std::move(sdata));
    }
    return inp;
}

// True if every group in `phase` is compatible with `cwGid` (no intergreen).
bool phaseCompatibleWith(const GPhase& phase, GId cwGid, const GConfig& cfg) {
    const auto& compat = cfg.compatibility.at(cwGid);
    return std::ranges::all_of(phase.groupIds,
                               [&](GId gid) { return compat.contains(gid); });
}

// Rough phase minimum ignoring transition green, used for heuristic selection.
int estimatePhaseMin(const GPhase& phase, const GConfig& cfg) {
    int m = 0;
    for (GId gid : phase.groupIds) m = std::max(m, cfg.groups[gid].tMin);
    return m;
}

// Among `compatIndices`, pick the phase whose estimated minimum is closest
// to `cwTMin`: prefer phases that already need >= cwTMin (free insertion),
// picking the smallest such; fall back to the largest below cwTMin.
int pickCompatiblePhase(const std::vector<int>& compatIndices,
                        const std::vector<GPhase>& phases, const GConfig& cfg,
                        int cwTMin) {
    int bestPi = compatIndices[0];
    int bestMin = estimatePhaseMin(phases[bestPi], cfg);

    for (int i = 1; i < (int)compatIndices.size(); ++i) {
        int pi = compatIndices[i];
        int phMin = estimatePhaseMin(phases[pi], cfg);
        bool curOk = bestMin >= cwTMin;
        bool newOk = phMin >= cwTMin;
        if (newOk && !curOk) {
            bestPi = pi;
            bestMin = phMin;
            continue;
        }
        if (!newOk && curOk) continue;
        if (newOk && phMin < bestMin) {
            bestPi = pi;
            bestMin = phMin;
            continue;
        }
        if (!newOk && phMin > bestMin) {
            bestPi = pi;
            bestMin = phMin;
        }
    }
    return bestPi;
}

// Insert a dedicated crosswalk phase at the position that minimises the net
// increase in transition overhead.
int findBestInsertPosition(const std::vector<GPhase>& phases,
                           const std::vector<GTransition>& transitions,
                           const GPhase& cwPhase, const GConfig& cfg) {
    int k = static_cast<int>(phases.size());
    int bestPos = 0;
    int bestOverhead = std::numeric_limits<int>::max();
    for (int pos = 0; pos < k; ++pos) {
        int next = (pos + 1) % k;
        int overhead =
            computeTransitionV2(phases[pos], cwPhase, cfg).makespan +
            computeTransitionV2(cwPhase, phases[next], cfg).makespan -
            transitions[pos].makespan;
        if (overhead < bestOverhead) {
            bestOverhead = overhead;
            bestPos = pos;
        }
    }
    return bestPos;
}

// Stage 4: insert all crosswalk groups from `cfg` into the vehicle-only cycle.
// Each crosswalk is placed in a compatible existing phase, or given a new
// dedicated phase at the cheapest insertion point. Returns nullopt if any
// crosswalk has no compatible existing phase AND creating a dedicated phase
// would still leave it incompatible with its neighbours (should not happen in
// a valid config, but guards against degenerate inputs).
std::optional<GCycle> insertCrosswalks(GCycle vehCycle, const GConfig& cfg,
                                       int T) {
    auto phases = std::move(vehCycle.phases);

    for (GId cwGid = 0; cwGid < (GId)cfg.groups.size(); ++cwGid) {
        const auto& cw = cfg.groups[cwGid];
        if (cw.kind != GKind::Crosswalk) continue;

        std::vector<int> compatIndices;
        for (int pi = 0; pi < (int)phases.size(); ++pi)
            if (phaseCompatibleWith(phases[pi], cwGid, cfg))
                compatIndices.push_back(pi);

        if (compatIndices.empty()) {
            // Build current transitions so we can score insertion positions.
            int k = static_cast<int>(phases.size());
            std::vector<GTransition> curTr(k);
            for (int i = 0; i < k; ++i)
                curTr[i] =
                    computeTransitionV2(phases[i], phases[(i + 1) % k], cfg);

            GPhase cwPhase{{cwGid}};
            int pos = findBestInsertPosition(phases, curTr, cwPhase, cfg);
            phases.insert(phases.begin() + pos + 1, cwPhase);
        } else {
            int bestPi =
                pickCompatiblePhase(compatIndices, phases, cfg, cw.tMin);
            phases[bestPi].groupIds.push_back(cwGid);
        }
    }

    // Recompute all transitions with crosswalks in place.
    int k = static_cast<int>(phases.size());
    std::vector<GTransition> transitions(k);
    for (int i = 0; i < k; ++i)
        transitions[i] =
            computeTransitionV2(phases[i], phases[(i + 1) % k], cfg);

    auto alloc = greedyMarginalAlloc(phases, transitions, cfg, T);
    if (!alloc) return std::nullopt;

    double D_full = totalDelay(*alloc, cfg, T);
    return GCycle{.phases = std::move(phases),
                  .transitions = std::move(transitions),
                  .alloc = std::move(*alloc),
                  .D = D_full};
}

// Stage 5: per-config optimisation with D_veh prune.
// `bestD` is the global best complete delay shared across parallel workers.
GCycle optimizeConfig(const std::vector<std::vector<GPhase>>& covers,
                      const GConfig& cfg, int T, std::atomic<double>& bestD) {
    GCycle best;

    for (const auto& cover : covers) {
        auto sequences = enumerateSequences(cover, cfg);
        for (auto& [order, transitions] : sequences) {
            std::vector<GPhase> phases;
            phases.reserve(order.size());
            for (int i : order) phases.push_back(cover[i]);

            auto vehAlloc = greedyMarginalAlloc(phases, transitions, cfg, T);
            if (!vehAlloc) continue;

            double D_veh = totalDelay(*vehAlloc, cfg, T);

            // Admissible prune: D_full >= D_veh, so if D_veh is already no
            // better than the known best, skip crosswalk insertion.
            if (D_veh >= bestD.load(std::memory_order_relaxed)) continue;

            GCycle vehCycle{.phases = phases,
                            .transitions = std::move(transitions),
                            .alloc = std::move(*vehAlloc),
                            .D = D_veh};

            auto fullCycle = insertCrosswalks(std::move(vehCycle), cfg, T);
            if (!fullCycle) continue;

            if (fullCycle->D < best.D) {
                best = std::move(*fullCycle);
                // Propagate to prune other workers.
                double prev = bestD.load(std::memory_order_relaxed);
                while (best.D < prev &&
                       !bestD.compare_exchange_weak(
                           prev, best.D, std::memory_order_relaxed)) {
                }
            }
        }
    }
    return best;
}

struct OptimizerRawResult {
    int cfgIdx = -1;
    GCycle cycle;
};

std::optional<OptimizerRawResult> pickBest(const std::vector<GConfig>& configs,
                                           int T, bool progressBar = true) {
    OptimizerRawResult globalBest;
    std::atomic<double> globalBestD{std::numeric_limits<double>::infinity()};
    std::mutex mu;
    std::atomic<int> done{0};
    const int total = static_cast<int>(configs.size());

    if (progressBar) {
        std::fprintf(stderr, "Optimizer: 0/%d (0%%)", total);
        std::fflush(stderr);
    }
    auto reportProgress = [&]() {
        if (!progressBar) return;
        int n = ++done;
        std::fprintf(stderr, "\rOptimizer: %d/%d (%d%%)", n, total,
                     n * 100 / total);
        std::fflush(stderr);
    };

    std::vector<std::future<void>> futures;
    futures.reserve(configs.size());

    for (int ci = 0; ci < total; ci++) {
        futures.push_back(std::async(std::launch::async, [&, ci]() {
            const auto& cfg = configs[ci];

            // Stage 1: movement-only phases for cover generation.
            auto allPhases = findAllCliques(cfg);
            std::vector<GPhase> validPhases;
            for (const auto& ph : allPhases) {
                if (std::ranges::none_of(ph.groupIds, [&](GId gid) {
                        return cfg.groups[gid].kind == GKind::Crosswalk;
                    }))
                    validPhases.push_back(ph);
            }
            if (validPhases.empty()) {
                reportProgress();
                return;
            }

            auto covers = generateCovers(validPhases, cfg);
            if (covers.empty()) {
                reportProgress();
                return;
            }

            GCycle result = optimizeConfig(covers, cfg, T, globalBestD);

            std::lock_guard<std::mutex> lock(mu);
            if (result.D < globalBest.cycle.D) {
                globalBest.cfgIdx = ci;
                globalBest.cycle = std::move(result);
            }
            reportProgress();
        }));
    }

    for (auto& f : futures) f.get();
    if (progressBar) {
        std::fprintf(stderr, "\n");
        std::fflush(stderr);
    }
    if (globalBest.cfgIdx < 0) return std::nullopt;
    return globalBest;
}

std::unordered_map<GId, Sequence> computeSequences(const GConfig& bestCfg,
                                                   const GCycle& best,
                                                   int cycleLength) {
    struct IntervalBlueprint {
        int onset;
        int duration;
    };

    std::vector<std::vector<IntervalBlueprint>> seqBps(
        bestCfg.groups.size());  // indexed by group
    std::vector<int> pendingWrap(bestCfg.groups.size(), 0);
    auto k = best.phases.size();
    int phaseOnset = 0;
    for (int i = 0; i < (int)k; ++i) {
        const auto& phase = best.phases[i];
        const auto& next = best.transitions[i];
        const auto& prev =
            i == 0 ? best.transitions[k - 1] : best.transitions[i - 1];
        auto alloc = best.alloc.tPhase[i];

        for (auto gid : phase.groupIds) {
            if (!prev.groups.at(gid).prevActive) {
                int onset = phaseOnset - prev.groups.at(gid).green;
                int duration = prev.groups.at(gid).green + alloc +
                               next.groups.at(gid).green;
                seqBps[gid].emplace_back(onset, duration);
            } else if (!seqBps[gid].empty()) {
                seqBps[gid].back().duration +=
                    alloc + next.groups.at(gid).green;
            } else {
                pendingWrap[gid] += alloc + next.groups.at(gid).green;
            }
        }

        phaseOnset += alloc + next.makespan;
    }

    for (GId gid = 0; gid < (int)bestCfg.groups.size(); ++gid) {
        if (pendingWrap[gid] <= 0) continue;
        if (!seqBps[gid].empty()) {
            seqBps[gid].back().duration += pendingWrap[gid];
        } else {
            // always active
            seqBps[gid].emplace_back(0, cycleLength);
        }
    }

    std::unordered_map<GId, Sequence> sequences;
    for (GId gid = 0; gid < (GId)bestCfg.groups.size(); ++gid) {
        const auto& g = bestCfg.groups[gid];
        int evAtomic = g.kind == GKind::Crosswalk ? 4 : 3;
        int apAtomic = g.kind == GKind::Crosswalk ? 0 : 1;

        std::vector<Interval> ivs = {};
        for (const auto& bp : seqBps[gid]) {
            // Normalize onset to [0, T): a window at −k equals one at T−k.
            int onset = ((bp.onset % cycleLength) + cycleLength) % cycleLength;
            if (onset == 0 && bp.duration == cycleLength) {
                sequences[gid] = AlwaysActive{};
                break;
            } else {
                ivs.emplace_back(onset, bp.duration, apAtomic, evAtomic);
            }
        }
        if (sequences.count(gid)) continue;  // already set to AlwaysActive
        SignalSequence seq{g.tMin};
        static_cast<void>(
            seq.setIntervals(  // NOLINT(bugprone-unused-return-value)
                std::move(ivs)));
        sequences[gid] = std::move(seq);
    }
    return sequences;
}

signal_optimizer::OptimizeResult compileResult(
    const GConfig& bestCfg, const GInput& inp,
    const std::unordered_map<GId, Sequence>& seqMap) {
    signal_optimizer::OptimizeResult out;
    for (GId gid = 0; gid < (GId)bestCfg.groups.size(); ++gid) {
        const auto& g = bestCfg.groups[gid];
        Sequence seq = seqMap.at(gid);

        if (g.kind == GKind::Crosswalk) {
            const auto& series = inp.series[g.seriesIdx];
            out.cgResults.push_back(signal_optimizer::CrosswalkGroupResult{
                .seriesId = series.id,
                .crosswalks = series.crosswalks,
                .sequence = std::move(seq),
            });
        } else {
            const auto& entry = inp.entries[g.entryIdx];
            signal_optimizer::MovementGroupResult mgr;
            mgr.isProtected = (g.kind == GKind::Protected);
            mgr.sequence = std::move(seq);
            for (int ci : g.clusterIdxs)
                mgr.clusters.push_back(entry.clusters[ci].id);
            for (const StreamId& sid : g.streams)
                if (const auto* mid = std::get_if<MovementId>(&sid))
                    mgr.movements.push_back(*mid);
            out.mgResults.push_back(std::move(mgr));
        }
    }
    return out;
}
}  // namespace

namespace signal_optimizer {
std::optional<OptimizeResult> optimize(NodeId node, const Graph& graph,
                                       const Demand& demand, int cycleLength) {
    auto inp = buildInput(node, graph, demand, cycleLength);
    if (inp.entries.empty() && inp.series.empty()) return std::nullopt;

    auto configs = generateConfigs(inp);
    if (configs.empty()) return std::nullopt;

    auto rawResult = pickBest(configs, inp.T);
    if (!rawResult) return std::nullopt;

    const GConfig& bestCfg = configs[rawResult->cfgIdx];
    const GCycle& best = rawResult->cycle;

    auto seqMap = computeSequences(bestCfg, best, cycleLength);
    return compileResult(bestCfg, inp, seqMap);
}
}  // namespace signal_optimizer
