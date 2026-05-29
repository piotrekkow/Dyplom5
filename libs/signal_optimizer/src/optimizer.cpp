#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
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

GCycle optimizeConfig(const std::vector<GPhase>& validPhases,
                      const std::vector<std::vector<GPhase>>& covers,
                      const GConfig& cfg, int T) {
    GCycle best;

    for (const auto& cover : covers) {
        auto sequences = enumerateSequences(cover, cfg);
        for (auto& [order, transitions] : sequences) {
            std::vector<GPhase> phases;
            phases.reserve(order.size());
            for (int i : order) phases.push_back(cover[i]);

            auto alloc = greedyMarginalAlloc(phases, transitions, cfg, T);
            if (!alloc) continue;

            double D = totalDelay(*alloc, cfg, T);
            GCycle candidate{.phases = phases,
                             .transitions = std::move(transitions),
                             .alloc = std::move(*alloc),
                             .D = D};

            if (candidate.D < best.D) best = std::move(candidate);
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

            auto validPhases = findAllCliques(cfg);
            if (validPhases.empty()) {
                reportProgress();
                return;
            }

            auto covers = generateCovers(validPhases, cfg);
            if (covers.empty()) {
                reportProgress();
                return;
            }

            GCycle result = optimizeConfig(validPhases, covers, cfg, T);

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

namespace {
// Forward distance a -> b around a cycle of length T, in [0, T).
inline int cycleAhead(int a, int b, int T) {
    int d = (b - a) % T;
    return d < 0 ? d + T : d;
}

// Widen every crosswalk green window as far as the per-pair intergreen allows,
// scanning the whole cycle. Windows are absolute: start in [0,T), end may
// exceed T (wrap). Vehicle windows are read-only. Every extension is measured
// against the pre-pass snapshot, so the result is independent of processing
// order even when two crosswalks conflict with each other.
void extendCrosswalkWindows(std::vector<int>& start, std::vector<int>& len,
                            const std::vector<bool>& present,
                            const GConfig& cfg, int T) {
    const auto s0 = start;
    const auto l0 = len;

    // Required clearance between ev losing green and ap gaining green.
    // -1 == compatible (no constraint). Crosswalk evac reserves +4 flashing
    // green, mirroring computeTransitionV2's `ig + 4`.
    auto clearance = [&](GId ev, GId ap) {
        int ig = cfg.intergreen[ev][ap];
        if (ig < 0) return -1;
        return cfg.groups[ev].kind == GKind::Crosswalk ? ig + 4 : ig;
    };

    for (GId cw = 0; cw < (GId)cfg.groups.size(); ++cw) {
        if (cfg.groups[cw].kind != GKind::Crosswalk || !present[cw]) continue;

        int fwd = T;  // how far cw's end may move forward (cw evac -> X appr)
        int bwd =
            T;  // how far cw's start may move backward (X evac -> cw appr)
        for (GId x = 0; x < (GId)cfg.groups.size(); ++x) {
            if (x == cw || !present[x]) continue;
            const int xEnd = s0[x] + l0[x];

            if (int clr = clearance(cw, x); clr >= 0)
                fwd =
                    std::min(fwd, cycleAhead(s0[cw] + l0[cw], s0[x], T) - clr);
            if (int clr = clearance(x, cw); clr >= 0)
                bwd = std::min(bwd, cycleAhead(xEnd, s0[cw], T) - clr);
        }

        // Don't let the two ends meet/cross; never shrink below the original.
        const int room = T - l0[cw];
        fwd = std::clamp(fwd, 0, room);
        bwd = std::clamp(bwd, 0, room - fwd);

        start[cw] = ((s0[cw] - bwd) % T + T) % T;
        len[cw] = l0[cw] + bwd + fwd;
    }
}
}  // namespace

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
    for (int i = 0; i < k; ++i) {
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

    for (GId gid = 0; gid < bestCfg.groups.size(); ++gid) {
        if (pendingWrap[gid] <= 0) continue;
        if (!seqBps[gid].empty()) {
            seqBps[gid].back().duration += pendingWrap[gid];
        } else {
            // always active
            seqBps[gid].emplace_back(0, cycleLength);
        }
    }

    std::vector<int> wStart(bestCfg.groups.size(), 0);
    std::vector<int> wLen(bestCfg.groups.size(), 0);
    std::vector<bool> present(bestCfg.groups.size(), false);
    for (GId gid = 0; gid < (GId)bestCfg.groups.size(); ++gid) {
        if (seqBps[gid].size() != 1) continue;  // skip empty / always-active
        const auto& bp = seqBps[gid].front();
        if (bp.onset == 0 && bp.duration == cycleLength) continue;
        wStart[gid] = ((bp.onset % cycleLength) + cycleLength) % cycleLength;
        wLen[gid] = bp.duration;
        present[gid] = true;
    }
    extendCrosswalkWindows(wStart, wLen, present, bestCfg, cycleLength);
    for (GId gid = 0; gid < (GId)bestCfg.groups.size(); ++gid)
        if (present[gid] && bestCfg.groups[gid].kind == GKind::Crosswalk)
            seqBps[gid].front() = {wStart[gid], wLen[gid]};

    std::unordered_map<GId, Sequence> sequences;
    for (GId gid = 0; gid < bestCfg.groups.size(); ++gid) {
        const auto& g = bestCfg.groups[gid];
        int evAtomic = g.kind == GKind::Crosswalk ? 4 : 3;
        int apAtomic = g.kind == GKind::Crosswalk ? 0 : 1;

        std::vector<Interval> ivs = {};
        for (const auto& bp : seqBps[gid]) {
            if (bp.onset == 0 && bp.duration == cycleLength) {
                sequences[gid] = AlwaysActive{};
                break;
            } else {
                ivs.emplace_back(bp.onset, bp.duration, apAtomic, evAtomic);
            }
        }
        SignalSequence seq{g.tMin};
        static_cast<void>(seq.setIntervals(std::move(ivs))); // NOLINT(bugprone-unused-return-value)
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
    GCycle best = std::move(rawResult->cycle);

    auto seqMap = computeSequences(bestCfg, best, cycleLength);
    return compileResult(bestCfg, inp, seqMap);
}
}  // namespace signal_optimizer
