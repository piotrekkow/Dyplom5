#include "config_generator.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <optional>

#include "road_network/graph/derived/turn_type.hpp"
#include "road_network/util/ids.hpp"
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
    size_t n) {
    std::vector<std::vector<std::vector<int>>> result;
    std::vector<std::vector<int>> currentPartition;

    if (n <= 0) return result;

    auto backtrack = [&](this auto& self, int start) -> void {
        if (start == n) {
            result.push_back(currentPartition);
            return;
        }
        std::vector<int> currentBlock;
        for (int i = start; i < n; ++i) {
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
    for (int ci = 0; ci < entry.clusters.size(); ++ci)
        for (auto& m : entry.clusters[ci].movements)
            if (m.turn == TurnType::THROUGH) return ci;
    return std::nullopt;
}

std::vector<StreamId> streamsAt(const GInput::EntryData& entry,
                                const std::vector<int>& clusterIdxs) {
    std::vector<StreamId> streams;
    size_t total = 0;
    for (const auto& cid : clusterIdxs)
        total += entry.clusters[cid].movements.size();
    streams.reserve(total);
    for (const auto& cid : clusterIdxs) {
        const auto& mvs = entry.clusters[cid].movements;
        for (const auto& m : mvs)
            streams.emplace_back(static_cast<StreamId>(m.id));
    }
    return streams;
}

std::vector<GCluster> groupClusters(const GInput::EntryData& entry,
                                    const std::vector<int>& clusterIdxs,
                                    bool isPermitted) {
    std::vector<GCluster> gc;
    gc.reserve(clusterIdxs.size());
    for (const auto& cid : clusterIdxs) {
        const auto& c = entry.clusters[cid];
        gc.emplace_back(c.Q,
                        isPermitted ? c.saturation.perm : c.saturation.prot);
    }
    return gc;
}

float maxYAt(const GInput::EntryData& entry,
             const std::vector<int>& clusterIdxs, bool isPermitted) {
    float maxY = 0.f;
    for (const auto& cid : clusterIdxs) {
        const auto& c = entry.clusters[cid];
        float saturation = isPermitted ? c.saturation.perm : c.saturation.prot;
        if (saturation < 1e-6f) return std::numeric_limits<float>::infinity();
        maxY = std::max(maxY, c.Q / saturation);
    }
    return maxY;
}

std::vector<std::vector<GGroup>> entryGroupLayouts(int entryIdx,
                                                   const GInput& inp) {
    const auto& entry = inp.entries[entryIdx];
    auto throughIdx = findThroughClusterIdx(entry);
    auto partitions = enumerateContiguousPartitions(entry.clusters.size());
    if (partitions.empty()) return {};
    std::vector<std::vector<GGroup>> result;
    result.reserve(partitions.size());

    auto makeLayout = [&](const auto& partition, auto kindFor) {
        std::vector<GGroup> pGroups;
        pGroups.reserve(partition.size());
        for (int gi = 0; gi < partition.size(); ++gi) {
            GKind kind = kindFor(gi);
            pGroups.push_back({
                .kind = kind,
                .entryIdx = entryIdx,
                .clusterIdxs = partition[gi],
                .clusters = groupClusters(entry, partition[gi],
                                          kind == GKind::Permitted),
                .streams = streamsAt(entry, partition[gi]),
                .maxY = maxYAt(entry, partition[gi], kind == GKind::Permitted),
                .tMin = 8,
            });
        }
        return pGroups;
    };

    for (const auto& partition : partitions) {
        if (throughIdx) {
            // entry has through: the clusters with through are required to
            // become a permitted group while others become protected
            auto layout = makeLayout(partition, [&](int gi) {
                return std::ranges::binary_search(partition[gi], *throughIdx)
                           ? GKind::Permitted
                           : GKind::Protected;
            });
            result.push_back(std::move(layout));
        } else {
            // entry has no through:
            // a) each partition gets k layouts, one per choice of permitted
            //    group
            int k = static_cast<int>(partition.size());
            for (int permIdx = 0; permIdx < k; ++permIdx) {
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
    for (StreamId s_ev : evac.streams) {
        auto it = inp.M.find(s_ev);
        if (it == inp.M.end()) continue;
        for (StreamId s_ap : appr.streams) {
            auto it2 = it->second.find(s_ap);
            if (it2 == it->second.end()) continue;
            if (appr.kind != GKind::Protected &&
                evac.kind != GKind::Protected) {
                auto pit = inp.P.find(s_ev);
                if (pit != inp.P.end() && pit->second.contains(s_ap)) continue;
            }
            max_ig = std::max(max_ig, it2->second.t_max);
        }
    }
    return max_ig;
}

}  // namespace

std::vector<GConfig> generateConfigs(const GInput& inp) {
    int nEntries = static_cast<int>(inp.entries.size());

    // Collect all valid per-entry layouts
    std::vector<std::vector<std::vector<GGroup>>> perEntryLayouts(nEntries);
    for (int ei = 0; ei < nEntries; ei++) {
        perEntryLayouts[ei] = entryGroupLayouts(ei, inp);
        if (perEntryLayouts[ei].empty()) return {};
    }

    // Cross-product across entries
    std::vector<GConfig> configs = {GConfig{}};

    for (int ei = 0; ei < nEntries; ei++) {
        std::vector<GConfig> next;
        next.reserve(configs.size() * perEntryLayouts[ei].size());
        for (const auto& cfg : configs) {
            for (const auto& layout : perEntryLayouts[ei]) {
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
            const auto& series = inp.series[si];
            GGroup g;
            g.id = static_cast<GId>(cfg.groups.size());
            g.kind = GKind::Crosswalk;
            g.entryIdx = -1;
            g.seriesIdx = si;
            for (CrosswalkId cid : series.crosswalks)
                g.streams.push_back(static_cast<StreamId>(cid));
            g.maxY = 0.f;
            g.tMin = series.minGreen;
            cfg.groups.push_back(std::move(g));
        }
    }

    // Compute intergreen matrices / compatibility graphs
    for (auto& cfg : configs) {
        const auto& cfgGr = cfg.groups;
        cfg.intergreen.assign(cfgGr.size(), std::vector<int>(cfgGr.size(), -1));
        for (int ev_i = 0; ev_i < cfgGr.size(); ++ev_i) {
            cfg.compatibility[ev_i];  // initialize compatibility graph with
                                      // empty set
            for (int ap_i = 0; ap_i < cfgGr.size(); ++ap_i) {
                if (ev_i == ap_i) continue;
                int maxIg = maxIntergreen(inp, cfgGr[ev_i], cfgGr[ap_i]);
                cfg.intergreen[ev_i][ap_i] = maxIg;
                if (maxIg < 0) {
                    cfg.compatibility[ev_i].insert(ap_i);
                    cfg.compatibility[ap_i].insert(ev_i);
                }
            }
        }
    }

    return configs;
}