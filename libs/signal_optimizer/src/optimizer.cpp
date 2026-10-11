#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <future>
#include <mutex>
#include <print>
#include <regulations/regulations.hpp>
#include <regulations/types/stream_kind.hpp>
#include <road_network/demand.hpp>
#include <road_network/graph.hpp>
#include <road_network/util/compute_helpers.hpp>
#include <signal_optimizer/optimizer.hpp>
#include <variant>

#include "config_generator.hpp"
#include "delay.hpp"
#include "green_time.hpp"
#include "stage_cover.hpp"
#include "stage_generator.hpp"
#include "stage_sequence.hpp"
#include "types.hpp"

namespace {
GInput buildInput(NodeId nodeId, const Graph& graph, const Demand& demand,
                  int T) {
    const auto& gdata = graph.data();
    const auto& gderived = graph.derived();
    const auto& regulations = graph.regulations();

    GInput inp{.entries = {}, .series = {}, .M = {}, .P = {}, .T = T};

    if (const auto* m = gderived.streamIntergreens().get(nodeId)) inp.M = *m;
    if (const auto* p = gderived.streamPermitted().get(nodeId)) inp.P = *p;
    const auto* node = gdata.nodes().get(nodeId);
    if (!node) return inp;

    for (EntryId eid : node->entries()) {
        const auto* entry = gdata.entries().get(eid);
        if (!entry || entry->clusters().empty()) continue;

        const auto* laneSat = demand.derived().laneSaturations().get(eid);

        GInput::EntryData edata{
            .id = eid, .heading = entry->params().heading, .clusters = {}};

        for (ClusterId cid : entry->clusters()) {
            const auto* cluster = gdata.clusters().get(cid);
            if (!cluster) continue;
            GInput::ClusterData cdata{
                .id = cid,
                .movements = {},
                .saturation = {},
                .Q = 0.0,
            };

            if (laneSat) {
                const auto& lanes = cluster->lanes();
                for (int li = lanes.firstIdx; li <= lanes.lastIdx(); ++li) {
                    if (li < 0 ||
                        static_cast<std::size_t>(li) >= laneSat->size())
                        continue;
                    cdata.saturation +=
                        (*laneSat)[static_cast<std::size_t>(li)];
                }
            }

            for (MovementId mid : cluster->movements()) {
                const auto* movement = gdata.movements().get(mid);
                if (!movement) continue;

                GInput::MovementData mdata{
                    .id = mid,
                    .turn = compute::turnKind(gdata, regulations, mid)};

                const auto* Q = demand.data().flows().get(mid);
                assert(Q);
                cdata.Q += Q->eqVehiclesPerHour();
                cdata.movements.push_back(mdata);
            }
            edata.clusters.push_back(std::move(cdata));
        }
        inp.entries.push_back(std::move(edata));
    }

    for (CrosswalkSeriesId csid : node->crosswalkSeries()) {
        const auto* cs = gdata.crosswalkSeries().get(csid);
        if (!cs || cs->crosswalks().empty()) continue;

        double speed = cs->params().criticalGreenSpeedOverride.value_or(
            regulations.kinematics(toStreamKind(cs->params().type))
                .evacSpeedCeil);
        int minGreen = std::max(
            4, static_cast<int>(
                   std::ceil(cs->params().criticalGreenPath.length() / speed)));
        GInput::SeriesData sdata{
            .id = csid, .crosswalks = cs->crosswalks(), .minGreen = minGreen};
        inp.series.push_back(std::move(sdata));
    }
    return inp;
}

// Counts where candidates die along the pipeline, to explain a nullopt result.
struct GPipelineStats {
    std::atomic<int> noCliques{0};
    std::atomic<int> noCovers{0};
    std::atomic<long> candidates{0};
    std::atomic<long> allocFail{0};
    std::atomic<long> infDelay{0};
};

GCycle optimizeConfig(const std::vector<std::vector<GStage>>& covers,
                      const GConfig& cfg, int T, const Regulations& regulations,
                      GPipelineStats& stats,
                      const std::function<bool()>& cancelled) {
    GCycle best;

    for (const auto& cover : covers) {
        if (cancelled && cancelled()) return best;
        auto candidates = enumerateSequences(cover, cfg, regulations);
        stats.candidates += static_cast<long>(candidates.size());
        for (auto& candidate : candidates) {
            if (cancelled && cancelled()) return best;
            auto alloc = greedyMarginalAlloc(candidate.stages,
                                             candidate.transitions, cfg, T);
            if (!alloc) {
                stats.allocFail++;
                continue;
            }

            double D = totalDelay(*alloc, cfg, T);
            if (!std::isfinite(D)) stats.infDelay++;
            if (D >= best.D) continue;

            control::StageSequence<GId> steps;
            steps.reserve(candidate.stages.size());
            for (std::size_t i = 0; i < candidate.stages.size(); ++i)
                steps.push_back(
                    {.stage = std::move(candidate.stages[i]),
                     .transition = std::move(candidate.transitions[i]),
                     .stageDuration = alloc->tStage[i]});

            best = GCycle{.steps = std::move(steps), .D = D};
        }
    }
    return best;
}

struct OptimizerRawResult {
    int cfgIdx = -1;
    GCycle cycle;
};

std::expected<OptimizerRawResult, signal_optimizer::OptimizeError> pickBest(
    const std::vector<GConfig>& configs, int T, const Regulations& regulations,
    signal_optimizer::OptimizationStats& out,
    const std::function<bool()>& cancelled,
    const std::function<void(int, int)>& progress, bool progressBar = true) {
    OptimizerRawResult globalBest;
    GPipelineStats stats;
    std::mutex mu;
    std::atomic<int> done{0};
    const int total = static_cast<int>(configs.size());
    if (progress) progress(0, total);

    if (progressBar) {
        std::print(stderr, "Optimizer: 0/{} (0%)", total);
        std::fflush(stderr);
    }
    auto reportProgress = [&]() {
        int n = ++done;
        if (progress) progress(n, total);
        if (!progressBar) return;
        std::print(stderr, "\rOptimizer: {}/{} ({}%)", n, total,
                   n * 100 / total);
        std::fflush(stderr);
    };

    std::vector<std::future<void>> futures;
    futures.reserve(configs.size());

    for (int ci = 0; ci < total; ci++) {
        futures.push_back(std::async(std::launch::async, [&, ci]() {
            if (cancelled && cancelled()) {
                reportProgress();
                return;
            }
            const auto& cfg = configs[static_cast<std::size_t>(ci)];

            auto validStages = findMaximalCliques(cfg);
            if (validStages.empty()) {
                stats.noCliques++;
                reportProgress();
                return;
            }

            auto covers = generateCovers(validStages, cfg);
            if (covers.empty()) {
                stats.noCovers++;
                reportProgress();
                return;
            }

            GCycle result =
                optimizeConfig(covers, cfg, T, regulations, stats, cancelled);

            std::lock_guard<std::mutex> lock(mu);
            if (result.D < globalBest.cycle.D) {
                globalBest.cfgIdx = ci;
                globalBest.cycle = std::move(result);
            }
            reportProgress();
        }));
    }

    for (auto& f : futures) f.get();
    out = {.configs = total,
           .sequences = stats.candidates.load(),
           .fullyInfeasible = stats.allocFail.load()};
    if (progressBar) {
        std::println(stderr, "");
        std::fflush(stderr);
    }
    std::println(stderr,
                 "Optimizer pipeline: {} configs ({} no stages, {} no covers), "
                 "{} sequences ({} alloc infeasible, {} infinite delay)",
                 total, stats.noCliques.load(), stats.noCovers.load(),
                 stats.candidates.load(), stats.allocFail.load(),
                 stats.infDelay.load());
    if (cancelled && cancelled())
        return std::unexpected(signal_optimizer::OptimizeError::Cancelled);
    if (globalBest.cfgIdx < 0) {
        std::println(stderr, "Optimizer: no feasible plan found");
        return std::unexpected(signal_optimizer::OptimizeError::NoFeasiblePlan);
    }
    return globalBest;
}

signal_optimizer::OptimizeResult compileResult(const GConfig& bestCfg,
                                               const GInput& inp,
                                               const GCycle& best,
                                               int cycleLength) {
    signal_optimizer::OptimizeResult out;
    out.cycleLength = cycleLength;

    for (const auto& g : bestCfg.groups) {
        if (g.kind == GKind::Crosswalk) {
            const auto& series =
                inp.series[static_cast<std::size_t>(g.seriesIdx)];
            out.cgResults.push_back(signal_optimizer::CrosswalkGroupResult{
                .seriesId = series.id,
                .crosswalks = series.crosswalks,
                .minGreen = g.tMin,
            });
        } else {
            const auto& entry =
                inp.entries[static_cast<std::size_t>(g.entryIdx)];
            signal_optimizer::MovementGroupResult mgr;
            mgr.isProtected = (g.kind == GKind::Protected);
            mgr.minGreen = g.tMin;
            for (int ci : g.clusterIdxs)
                mgr.clusters.push_back(
                    entry.clusters[static_cast<std::size_t>(ci)].id);
            for (const StreamId& sid : g.streams)
                if (const auto* mid = std::get_if<MovementId>(&sid))
                    mgr.movements.push_back(*mid);
            out.mgResults.push_back(std::move(mgr));
        }
    }

    out.stages = best.steps;

    return out;
}
}  // namespace

namespace signal_optimizer {
std::expected<OptimizeResult, OptimizeError> optimize(
    NodeId node, const Graph& graph, const Demand& demand, int cycleLength,
    std::function<bool()> cancelled,
    std::function<void(int completed, int total)> progress) {
    try {
        if (!graph.data().nodes().alive(node))
            return std::unexpected(OptimizeError::InvalidNode);
        if (cycleLength <= 0)
            return std::unexpected(OptimizeError::InvalidCycleLength);
        if (cancelled && cancelled())
            return std::unexpected(OptimizeError::Cancelled);

        const Node* sourceNode = graph.data().nodes().get(node);
        for (EntryId entryId : sourceNode->entries()) {
            const Entry* entry = graph.data().entries().get(entryId);
            if (entry == nullptr) continue;
            for (ClusterId clusterId : entry->clusters()) {
                const Cluster* cluster = graph.data().clusters().get(clusterId);
                if (cluster == nullptr) continue;
                for (MovementId movementId : cluster->movements())
                    if (demand.data().flows().get(movementId) == nullptr)
                        return std::unexpected(
                            OptimizeError::MissingDemandData);
            }
        }

        const auto& regulations = graph.regulations();
        auto inp = buildInput(node, graph, demand, cycleLength);
        if (inp.entries.empty() && inp.series.empty())
            return std::unexpected(OptimizeError::NoStreams);
        if (cancelled && cancelled())
            return std::unexpected(OptimizeError::Cancelled);

        auto configs = generateConfigs(inp);
        if (configs.empty())
            return std::unexpected(OptimizeError::NoConfigurations);
        if (cancelled && cancelled())
            return std::unexpected(OptimizeError::Cancelled);

        OptimizationStats stats;
        auto rawResult =
            pickBest(configs, inp.T, regulations, stats, cancelled, progress);
        if (!rawResult) return std::unexpected(rawResult.error());

        const GConfig& bestCfg =
            configs[static_cast<std::size_t>(rawResult->cfgIdx)];
        auto result =
            compileResult(bestCfg, inp, rawResult->cycle, cycleLength);
        result.stats = stats;
        return result;
    } catch (const std::exception&) {
        return std::unexpected(OptimizeError::InternalFailure);
    } catch (...) {
        return std::unexpected(OptimizeError::InternalFailure);
    }
}
}  // namespace signal_optimizer
