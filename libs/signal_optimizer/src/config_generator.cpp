#include "config_generator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>
#include <regulations/types/turn_kind.hpp>
#include <road_network/util/compute_helpers.hpp>
#include <road_network/util/ids.hpp>

#include "types.hpp"

namespace {
// all contiguous partitions of vector<int>
// example:
// input: {1, 2, 3, 4}
// output:
// {{1}, {2}, {3}, {4}},
// {{1, 2}, {3}, {4}},
// {{1}, {2, 3}, {4}},
// {{1}, {2}, {3, 4}},
// {{1, 2, 3}, {4}},
// {{1}, {2, 3, 4}},
// {{1, 2, 3, 4}}
std::vector<std::vector<std::vector<int>>> enumerateContiguousPartitions(
    std::size_t n) {
    std::vector<std::vector<std::vector<int>>> result;
    std::vector<std::vector<int>> currentPartition;

    if (n <= 0) return result;

    auto backtrack = [&](this auto& self, int start) -> void {
        if (start == static_cast<int>(n)) {
            result.push_back(currentPartition);
            return;
        }
        std::vector<int> currentBlock;
        for (int i = start; i < static_cast<int>(n); ++i) {
            currentBlock.push_back(i);
            currentPartition.push_back(currentBlock);
            self(i + 1);
            currentPartition.pop_back();
        }
    };
    backtrack(0);
    return result;
}

std::optional<int> findThroughClusterIdx(const GInput::EntryData& entry) {
    for (int ci = 0; ci < static_cast<int>(entry.clusters.size()); ++ci)
        for (auto& m : entry.clusters[static_cast<std::size_t>(ci)].movements)
            if (m.turn == TurnKind::THROUGH) return ci;
    return std::nullopt;
}

std::vector<StreamId> streamsAt(const GInput::EntryData& entry,
                                const std::vector<int>& clusterIdxs) {
    std::vector<StreamId> streams;
    std::size_t total = 0;
    for (const auto& cid : clusterIdxs)
        total += entry.clusters[static_cast<std::size_t>(cid)].movements.size();
    streams.reserve(total);
    for (const auto& cid : clusterIdxs) {
        const auto& mvs =
            entry.clusters[static_cast<std::size_t>(cid)].movements;
        for (const auto& m : mvs) streams.emplace_back(m.id);
    }
    return streams;
}

std::vector<GCluster> groupClusters(const GInput::EntryData& entry,
                                    const std::vector<int>& clusterIdxs,
                                    bool isPermitted) {
    std::vector<GCluster> gc;
    gc.reserve(clusterIdxs.size());
    for (const auto& cid : clusterIdxs) {
        const auto& c = entry.clusters[static_cast<std::size_t>(cid)];
        gc.emplace_back(c.Q,
                        isPermitted ? c.saturation.perm : c.saturation.prot);
    }
    return gc;
}

double maxYAt(const GInput::EntryData& entry,
              const std::vector<int>& clusterIdxs, bool isPermitted) {
    double maxY = 0.0;
    for (const auto& cid : clusterIdxs) {
        const auto& c = entry.clusters[static_cast<std::size_t>(cid)];
        double saturation = isPermitted ? c.saturation.perm : c.saturation.prot;
        if (saturation < 1e-6) return std::numeric_limits<double>::infinity();
        maxY = std::max(maxY, c.Q / saturation);
    }
    return maxY;
}

std::vector<std::vector<GGroup>> entryGroupLayouts(int entryIdx,
                                                   const GInput& inp) {
    const auto& entry = inp.entries[static_cast<std::size_t>(entryIdx)];
    auto throughIdx = findThroughClusterIdx(entry);
    auto partitions = enumerateContiguousPartitions(entry.clusters.size());
    if (partitions.empty()) return {};
    std::vector<std::vector<GGroup>> result;
    result.reserve(partitions.size());

    auto makeLayout = [&](const auto& partition, auto kindFor) {
        std::vector<GGroup> pGroups;
        pGroups.reserve(partition.size());
        for (int gi = 0; gi < static_cast<int>(partition.size()); ++gi) {
            GKind kind = kindFor(gi);
            pGroups.push_back({
                .kind = kind,
                .entryIdx = entryIdx,
                .clusterIdxs = partition[static_cast<std::size_t>(gi)],
                .clusters = groupClusters(
                    entry, partition[static_cast<std::size_t>(gi)],
                    kind == GKind::Permitted),
                .streams =
                    streamsAt(entry, partition[static_cast<std::size_t>(gi)]),
                .maxY = maxYAt(entry, partition[static_cast<std::size_t>(gi)],
                               kind == GKind::Permitted),
                .tMin = 8,
            });
        }
        return pGroups;
    };

    // A group may only run permitted if that yields finite flow ratios —
    // S.perm == 0 marks streams that must not run on a permitted signal.
    auto permFeasible = [&](const std::vector<int>& clusterIdxs) {
        return std::isfinite(maxYAt(entry, clusterIdxs, true));
    };

    for (const auto& partition : partitions) {
        if (throughIdx) {
            // entry has through: the clusters with through are required to
            // become a permitted group while others become protected
            auto layout = makeLayout(partition, [&](int gi) {
                const auto& block = partition[static_cast<std::size_t>(gi)];
                if (!std::ranges::binary_search(block, *throughIdx))
                    return GKind::Protected;
                return permFeasible(block) ? GKind::Permitted
                                           : GKind::Protected;
            });
            result.push_back(std::move(layout));
        } else {
            // entry has no through:
            // a) each partition gets k layouts, one per choice of permitted
            //    group (skipping groups that cannot run permitted)
            int k = static_cast<int>(partition.size());
            for (int permIdx = 0; permIdx < k; ++permIdx) {
                if (!permFeasible(partition[static_cast<std::size_t>(permIdx)]))
                    continue;
                auto layout = makeLayout(partition, [permIdx](int gi) {
                    return gi == permIdx ? GKind::Permitted : GKind::Protected;
                });
                result.push_back(std::move(layout));
            }
            // b) each partition gets layout with all protected groups
            auto layout =
                makeLayout(partition, [](int) { return GKind::Protected; });
            result.push_back(std::move(layout));
        }
    }
    return result;
}

int maxIntergreen(const GInput& inp, const GGroup& evac, const GGroup& appr) {
    int max_ig = -1;
    bool evacProtected = evac.kind == GKind::Protected;
    bool apprProtected = appr.kind == GKind::Protected;
    for (StreamId s_ev : evac.streams)
        for (StreamId s_ap : appr.streams)
            max_ig = std::max(max_ig, compute::resolveStreamIntergreen(
                                          inp.M, inp.P, s_ev, s_ap,
                                          evacProtected, apprProtected));
    return max_ig;
}

}  // namespace

std::vector<GConfig> generateConfigs(const GInput& inp) {
    int nEntries = static_cast<int>(inp.entries.size());

    // Collect all valid per-entry layouts
    std::vector<std::vector<std::vector<GGroup>>> perEntryLayouts(
        static_cast<std::size_t>(nEntries));
    for (int ei = 0; ei < nEntries; ei++) {
        perEntryLayouts[static_cast<std::size_t>(ei)] =
            entryGroupLayouts(ei, inp);
        if (perEntryLayouts[static_cast<std::size_t>(ei)].empty()) return {};
    }

    // Cross-product across entries
    std::vector<GConfig> configs = {GConfig{}};

    for (int ei = 0; ei < nEntries; ei++) {
        std::vector<GConfig> next;
        next.reserve(configs.size() *
                     perEntryLayouts[static_cast<std::size_t>(ei)].size());
        for (const auto& cfg : configs) {
            for (const auto& layout :
                 perEntryLayouts[static_cast<std::size_t>(ei)]) {
                GConfig newCfg = cfg;
                for (GGroup g : layout) {
                    g.id = static_cast<GId>(newCfg.groups.size());
                    newCfg.groups.push_back(std::move(g));
                }
                next.push_back(std::move(newCfg));
            }
        }
        configs = std::move(next);
    }

    // Append crosswalk groups (identical across all configs)
    int nSeries = static_cast<int>(inp.series.size());
    for (auto& cfg : configs) {
        for (int si = 0; si < nSeries; si++) {
            const auto& series = inp.series[static_cast<std::size_t>(si)];
            GGroup g;
            g.id = static_cast<GId>(cfg.groups.size());
            g.kind = GKind::Crosswalk;
            g.entryIdx = -1;
            g.seriesIdx = si;
            for (CrosswalkId cid : series.crosswalks) g.streams.push_back(cid);
            g.maxY = 0.0;
            g.tMin = series.minGreen;
            cfg.groups.push_back(std::move(g));
        }
    }

    // Compute intergreen matrices / compatibility graphs
    for (auto& cfg : configs) {
        const auto& cfgGr = cfg.groups;
        auto n = static_cast<GId>(cfgGr.size());
        for (GId ev_i = 0; ev_i < n; ++ev_i) {
            cfg.compatibility[ev_i];  // initialize compatibility graph with
                                      // empty set
            for (GId ap_i = ev_i + 1; ap_i < n; ++ap_i) {
                int maxIgEvAp =
                    maxIntergreen(inp, cfgGr[static_cast<std::size_t>(ev_i)],
                                  cfgGr[static_cast<std::size_t>(ap_i)]);
                int maxIgApEv =
                    maxIntergreen(inp, cfgGr[static_cast<std::size_t>(ap_i)],
                                  cfgGr[static_cast<std::size_t>(ev_i)]);
                // Stored clamped to >=0: control::computeTransitionWeighted
                // treats a missing/zero entry as "no extra clearance beyond
                // the groups' own aspect-transition timing", which is exactly
                // what the -1 "compatible" sentinel means here.
                cfg.intergreen.set(ev_i, ap_i, std::max(maxIgEvAp, 0),
                                   std::max(maxIgApEv, 0));
                if (maxIgEvAp < 0 && maxIgApEv < 0) {
                    cfg.compatibility[ev_i].insert(ap_i);
                    cfg.compatibility[ap_i].insert(ev_i);
                }
            }
        }
    }

    return configs;
}
