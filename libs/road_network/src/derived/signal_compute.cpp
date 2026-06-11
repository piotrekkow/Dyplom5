#include "signal_compute.hpp"

#include <algorithm>

#include "road_network/graph/graph_derived.hpp"
#include "road_network/signal/signal_data.hpp"
#include "road_network/util/ids.hpp"

namespace signal_compute {
IntergreenMatrix intergreenMatrix(const GraphDerived& graphDerived,
                                  const SignalData& signal,
                                  SignalConfigId id) {
    IntergreenMatrix result;
    const auto* cfg = signal.configs().get(id);
    if (!cfg) return result;
    const auto& groups = cfg->signalGroups();
    const auto* intersection = signal.intersections().get(cfg->at());
    if (!intersection) return result;
    const auto& nodes = intersection->nodes();

    // Per-node maps only hold intra-node stream pairs; pairs spanning two
    // nodes of the intersection have no geometric conflict and therefore no
    // intergreen constraint.
    auto permitted = [&](StreamId a, StreamId b) {
        for (NodeId node : nodes) {
            const auto* permMap = graphDerived.streamPermitted().get(node);
            if (!permMap) continue;
            auto it = permMap->find(a);
            if (it != permMap->end() && it->second.count(b)) return true;
        }
        return false;
    };

    auto groupStreams = [&](SignalGroupId sgId) {
        const auto* g = signal.signalGroups().get(sgId);
        return std::visit([](const auto& x) { return x.streams(); }, *g);
    };

    auto eval = [&](SignalGroupId evacGroupId, SignalGroupId apprGroupId) {
        int maxTime = -1;
        auto isProtectedMovement = [&](SignalGroupId gid) -> bool {
            const auto* sg = signal.signalGroups().get(gid);
            if (!sg) return false;
            if (const auto* mg = std::get_if<MovementSignalGroup>(sg))
                return mg->isProtected();
            return false;
        };

        bool evacProtected = isProtectedMovement(evacGroupId);
        bool apprProtected = isProtectedMovement(apprGroupId);

        for (auto evStr : groupStreams(evacGroupId)) {
            for (auto apStr : groupStreams(apprGroupId)) {
                for (NodeId node : nodes) {
                    const auto* igMap =
                        graphDerived.streamIntergreen().get(node);
                    if (!igMap) continue;
                    auto evacIt = igMap->find(evStr);
                    if (evacIt == igMap->end()) continue;
                    auto apprIt = evacIt->second.find(apStr);
                    if (apprIt == evacIt->second.end()) continue;

                    bool skip = false;
                    if (!evacProtected && !apprProtected) {
                        // Both are non‑protected → use permitted map to skip
                        if (permitted(evStr, apStr) && permitted(apStr, evStr))
                            skip = true;
                    }

                    if (!skip) {
                        maxTime = std::max(maxTime, apprIt->second.t_max);
                    }
                }
            }
        }
        return maxTime;
    };

    for (auto evacGroup : groups) {
        for (auto apprGroup : groups) {
            if (evacGroup == apprGroup) continue;
            int entry = eval(evacGroup, apprGroup);
            if (entry >= 0) {
                result[evacGroup][apprGroup] = entry;
            }
        }
    }

    return result;
}
}  // namespace signal_compute