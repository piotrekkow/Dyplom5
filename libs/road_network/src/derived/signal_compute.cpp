#include "signal_compute.hpp"

#include <algorithm>

#include "road_network/graph/graph_derived.hpp"
#include "road_network/signal/signal_data.hpp"
#include "road_network/util/ids.hpp"

namespace signal_compute {
IntergreenMatrix intergreenMatrix(const GraphDerived& graphDerived,
                                  const SignalData& signal, NodeId id) {
    IntergreenMatrix result;
    const auto* nodeSig = signal.nodeSignals().get(id);
    if (!nodeSig) return result;
    auto& groups = nodeSig->signalGroups;
    const int n = static_cast<int>(groups.size());

    const auto* permMap = graphDerived.streamPermitted().get(id);
    const auto* igMap = graphDerived.streamIntergreen().get(id);
    if (!permMap || !igMap) return result;

    auto permitted = [&](StreamId a, StreamId b) {
        auto it = permMap->find(a);
        return it != permMap->end() && it->second.count(b);
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