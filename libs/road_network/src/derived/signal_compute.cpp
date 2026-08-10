#include "signal_compute.hpp"

#include <common/overloaded.hpp>
#include <regulations/regulations.hpp>

#include "road_network/graph.hpp"
#include "road_network/signal.hpp"
#include "road_network/util/compute_helpers.hpp"

namespace signal_compute {
IntergreenMatrix intergreenMatrix(const GraphDerived& graphDerived,
                                  const Signal& signal,
                                  const Regulations& regulations,
                                  NodeId nodeId) {
    IntergreenMatrix result;
    const auto& groupsGlobal = signal.data().signalGroups();
    const auto* ng = signal.data().nodeGroups().get(nodeId);
    if (!ng) return result;  // no groups at this node yet
    const auto& nodeGroups = *ng;

    // Per-node maps only hold intra-node stream pairs; pairs spanning two nodes
    // of the intersection have no geometric conflict and thus no constraint.
    static const StreamMatrix<bool> kNoPermitted;
    const auto* permittedMap = graphDerived.streamPermitted().get(nodeId);
    const StreamMatrix<bool>& permitted =
        permittedMap ? *permittedMap : kNoPermitted;

    auto groupStreams = [&](SignalGroupId sgId) {
        const auto* g = groupsGlobal.get(sgId);
        return std::visit([](const auto& x) { return x.streams(); }, *g);
    };

    auto isProtectedMovement = [&](SignalGroupId gid) -> bool {
        const auto* res = signal.derived().protectionResolutions().get(gid);
        return res->isProtected();
    };

    auto eval = [&](SignalGroupId evacGroupId, SignalGroupId apprGroupId) {
        int maxTime = -1;
        bool evacProtected = isProtectedMovement(evacGroupId);
        bool apprProtected = isProtectedMovement(apprGroupId);
        const auto* igMap = graphDerived.streamIntergreens().get(nodeId);
        assert(igMap);

        for (auto evStr : groupStreams(evacGroupId))
            for (auto apStr : groupStreams(apprGroupId))
                maxTime = std::max(maxTime, compute::resolveStreamIntergreen(
                                                *igMap, permitted, evStr, apStr,
                                                evacProtected, apprProtected));
        return maxTime;
    };

    auto evacMargin = [&](SignalGroupId evacGroupId) {
        const auto* g = groupsGlobal.get(evacGroupId);
        SignalKind kind =
            std::visit([](const auto& x) { return x.params().kind; }, *g);
        return regulations.aspects(kind).additionalIntergreenEvac;
    };

    for (size_t i = 0; i < nodeGroups.size(); ++i) {
        for (size_t j = i + 1; j < nodeGroups.size(); ++j) {
            auto evacGroup = nodeGroups[i];
            auto apprGroup = nodeGroups[j];
            int evacT = eval(evacGroup, apprGroup);
            int apprT = eval(apprGroup, evacGroup);
            if (evacT >= 0) {
                evacT += evacMargin(evacGroup);
                apprT += evacMargin(apprGroup);
                result.set(evacGroup, apprGroup, evacT, apprT);
            }
        }
    }

    return result;
}

ApproachMatrix approachMatrix(const GraphDerived& graphDerived,
                              const Signal& signal, NodeId nodeId,
                              const IntergreenMatrix& intergreen) {
    ApproachMatrix result;
    const auto* ng = signal.data().nodeGroups().get(nodeId);
    if (!ng) return result;
    const auto& nodeGroups = *ng;
    const auto* approachStreams = graphDerived.streamApproach().get(nodeId);
    if (!approachStreams) return result;

    const auto& groupsGlobal = signal.data().signalGroups();
    auto groupStreams = [&](SignalGroupId g) {
        return std::visit([](const auto& x) { return x.streams(); },
                          *groupsGlobal.get(g));
    };
    auto isVehicleGroup = [&](SignalGroupId g) {
        return std::holds_alternative<MovementSignalGroup>(
            *groupsGlobal.get(g));
    };

    // cap(i, j): the largest lag start[i] - start[j] the approach rule allows,
    // or -1 when the direction carries no constraint. Active only when the pair
    // is permitted to run together (no intergreen governs them) and the leader
    // j is a vehicle group whose traffic actually approaches i. The floor (min)
    // over conflicting stream pairs takes the soonest arrival.
    auto cap = [&](SignalGroupId gi, SignalGroupId gj) -> int {
        if (!isVehicleGroup(gj)) return -1;  // pedestrian leader: no deadline
        if (intergreen.find(gi, gj))
            return -1;  // conflicting: intergreen rules
        int best = -1;
        for (auto si : groupStreams(gi))
            for (auto sj : groupStreams(gj))
                if (const int* a = approachStreams->find(si, sj))
                    best = best < 0 ? *a : std::min(best, *a);
        return best;  // -1 if the groups never meet geometrically
    };

    for (std::size_t i = 0; i < nodeGroups.size(); ++i)
        for (std::size_t j = i + 1; j < nodeGroups.size(); ++j) {
            int cij = cap(nodeGroups[i], nodeGroups[j]);
            int cji = cap(nodeGroups[j], nodeGroups[i]);
            if (cij >= 0 || cji >= 0)
                result.set(nodeGroups[i], nodeGroups[j], cij, cji);
        }
    return result;
}

std::unordered_map<SignalGroupId, ProtectionResolution> protectionResolution(
    const SignalData& signal, const GraphData& graph,
    const Regulations& regulations, NodeId nodeId) {
    std::unordered_map<EntryId, std::optional<MovementId>> throughMovement;
    for (auto eid : graph.nodes().get(nodeId)->entries()) {
        throughMovement[eid] = std::nullopt;
        for (auto mid : graph.entries().get(eid)->movements()) {
            if (compute::turnKind(graph, regulations, mid) == TurnKind::THROUGH)
                throughMovement[eid] = mid;
        }
    }

    std::unordered_map<SignalGroupId, ProtectionResolution> result;
    const auto* nodeGroups = signal.nodeGroups().get(nodeId);
    if (!nodeGroups) return result;  // no groups at this node yet
    for (auto gid : *nodeGroups) {
        const auto* sg = signal.signalGroups().get(gid);
        std::visit(
            overloaded{
                [&](const MovementSignalGroup& msg) {
                    auto entry = msg.entry();
                    if (!throughMovement[entry]) {
                        result[gid] = {
                            .forced = false,
                            .requested = msg.params().requestProtected};
                    } else {
                        bool isThroughGroup = false;
                        for (auto sid : msg.streams()) {
                            const auto* mid = std::get_if<MovementId>(&sid);
                            if (mid && throughMovement[entry] == *mid) {
                                isThroughGroup = true;
                                break;
                            }
                        }
                        result[gid] = {
                            .forced = !isThroughGroup,
                            .requested = msg.params().requestProtected};
                    }
                },
                [&](const CrosswalkSignalGroup&) {
                    result[gid] = {.forced = false, .requested = false};
                },
            },
            *sg);
    }

    return result;
}

}  // namespace signal_compute