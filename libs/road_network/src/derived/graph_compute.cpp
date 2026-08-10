#include "graph_compute.hpp"

#include <algorithm>
#include <cmath>
#include <common/overloaded.hpp>
#include <limits>
#include <regulations/regulations.hpp>
#include <regulations/types/stream_kind.hpp>

#include "road_network/graph/derived/graph_geometry.hpp"
#include "road_network/graph.hpp"
#include "road_network/graph/graph_data.hpp"
#include "road_network/util/compute_helpers.hpp"

namespace graph_compute {
namespace {

//
//  ConflictMap
//
std::unordered_map<MovementId, std::vector<Polyline>> movementGeometries(
    const GraphData& data, NodeId id) {
    std::unordered_map<MovementId, std::vector<Polyline>> result;
    const auto* node = data.nodes().get(id);
    if (!node) return result;

    for (auto eId : node->entries()) {
        auto* e = data.entries().get(eId);
        for (auto mId : e->movements()) {
            result[mId] = graph_geometry::movementPolylines(data, mId);
        }
    }
    return result;
}

std::unordered_map<CrosswalkId, CrosswalkGeometry> crosswalkGeometries(
    const GraphData& data, NodeId id) {
    std::unordered_map<CrosswalkId, CrosswalkGeometry> result;
    auto* node = data.nodes().get(id);
    if (!node) return result;

    for (auto csId : node->crosswalkSeries()) {
        for (auto cId : data.crosswalkSeries().get(csId)->crosswalks()) {
            result[cId] = data.crosswalks().get(cId)->geometry();
        }
    }
    return result;
}

std::vector<ConflictPoint> movMovPoints(
    const std::vector<Polyline>& evacPaths,
    const std::vector<Polyline>& approachPaths) {
    std::vector<ConflictPoint> cpVec;
    for (const auto& evacPath : evacPaths) {
        for (const auto& approachPath : approachPaths) {
            auto pathIntersection = evacPath.lastIntersection(approachPath);
            if (!pathIntersection.has_value()) continue;
            auto pi = pathIntersection.value();

            cpVec.emplace_back(pi.point, pi.distanceA, pi.distanceB);
        }
    }
    return cpVec;
}

std::vector<ConflictPoint> movCrossPoints(
    const std::vector<Polyline>& evacPaths,
    const CrosswalkGeometry& apprCrosswalk) {
    std::vector<ConflictPoint> cpVec;
    for (const auto& evacPath : evacPaths) {
        // The crosswalk is a band between its anchor and split lines; the
        // evacuating vehicle has cleared it at the farther edge crossing.
        auto anchor = evacPath.firstIntersection(apprCrosswalk.anchor);
        auto split = evacPath.firstIntersection(apprCrosswalk.split);
        const auto& furthest =
            !split || (anchor && anchor->distanceA > split->distanceA) ? anchor
                                                                       : split;
        if (!furthest) continue;
        cpVec.push_back({.position = furthest->point,
                         .evacDistance = furthest->distanceA,
                         .apprDistance = 0.0});
    }
    return cpVec;
}

std::vector<ConflictPoint> crossMovPoints(
    const CrosswalkGeometry& evacCrosswalk,
    const std::vector<Polyline>& approachPaths) {
    std::vector<ConflictPoint> cpVec;
    const double maxCrosswalkLineLength =
        std::max(evacCrosswalk.anchor.length(), evacCrosswalk.split.length());
    for (const auto& approachPath : approachPaths) {
        // The approaching vehicle meets the band at the nearer edge crossing;
        // the evacuating pedestrian worst-case clears the full span.
        auto anchor = approachPath.firstIntersection(evacCrosswalk.anchor);
        auto split = approachPath.firstIntersection(evacCrosswalk.split);
        const auto& closest =
            !split || (anchor && anchor->distanceA < split->distanceA) ? anchor
                                                                       : split;
        if (!closest) continue;
        cpVec.push_back({.position = closest->point,
                         .evacDistance = maxCrosswalkLineLength,
                         .apprDistance = closest->distanceA});
    }
    return cpVec;
}
}  // namespace

StreamMatrix<std::vector<ConflictPoint>> conflictMap(const GraphData& data,
                                                     NodeId id) {
    StreamMatrix<std::vector<ConflictPoint>> cm;

    auto movGeometries = movementGeometries(data, id);
    auto crossGeometries = crosswalkGeometries(data, id);

    /// MOVEMENT <-> MOVEMENT
    for (auto a = movGeometries.begin(); a != movGeometries.end(); ++a) {
        auto* aMov = data.movements().get(a->first);
        if (!aMov) continue;
        for (auto b = std::next(a); b != movGeometries.end(); ++b) {
            auto* bMov = data.movements().get(b->first);
            if (!bMov) continue;
            if (aMov->from() == bMov->from()) continue;
            auto ab = movMovPoints(a->second, b->second);
            auto ba = movMovPoints(b->second, a->second);
            if (ab.empty() && ba.empty()) continue;  // paths never cross
            cm.set(a->first, b->first, std::move(ab), std::move(ba));
        }
    }

    /// MOVEMENT <-> CROSSWALK
    for (const auto& [movId, movGeo] : movGeometries) {
        for (const auto& [crossId, crossGeo] : crossGeometries) {
            auto movToCross = movCrossPoints(movGeo, crossGeo);
            auto crossToMov = crossMovPoints(crossGeo, movGeo);
            if (movToCross.empty() && crossToMov.empty()) continue;
            cm.set(movId, crossId, std::move(movToCross),
                   std::move(crossToMov));
        }
    }

    return cm;
}
}  // namespace graph_compute

//
// StreamIntergreenMaps
//

namespace {
struct StreamParams {
    StreamKind kind;
    double v_evac;  // evacuation speed (v_e)
    double v_appr;  // approach speed (v_d). Crosswalk conflicts always carry
                    // s_appr == 0 (see movCrossPoints), so this only needs
                    // to be nonzero to keep t_appr's s_appr/v_appr out of 0/0.
    double l_evac;  // clearing length  (l_p)
    double t_y;     // yellow time
};

StreamParams streamParams(const Graph& graph, StreamId id) {
    const auto& data = graph.data();
    const auto& reg = graph.regulations();
    return std::visit(
        overloaded{
            [&](MovementId mid) -> StreamParams {
                const auto* m = data.movements().get(mid);
                double eLimit =
                    data.entries().get(m->from())->params().speedLimit;
                double xLimit = data.exits().get(m->to())->params().speedLimit;
                auto turn = compute::turnKind(data, reg, mid);
                auto k = reg.kinematics(m->params().kind(), turn);
                double v_appr =
                    k.approachSpeedFloor
                        ? std::max({eLimit, xLimit, *k.approachSpeedFloor})
                        : std::max(eLimit, xLimit);
                return {.kind = m->params().kind(),
                        .v_evac = std::min({eLimit, xLimit, k.evacSpeedCeil}),
                        .v_appr = v_appr,
                        .l_evac = k.clearingLength,
                        .t_y = k.yellowTime};
            },
            [&](CrosswalkId cid) -> StreamParams {
                const auto* cw = data.crosswalks().get(cid);
                const auto& cs = data.crosswalkSeries().get(cw->parent());
                auto skind = toStreamKind(cs->params().type);
                auto k = reg.kinematics(skind);
                return {.kind = skind,
                        .v_evac = k.evacSpeedCeil,
                        .v_appr = -1.0,  // any nonzero value; s_appr is 0
                        .l_evac = k.clearingLength,
                        .t_y = k.yellowTime};
            }},
        id);
}
}  // namespace
namespace graph_compute {
StreamIntergreenMaps streamIntergreenMaps(const Graph& graph, NodeId id) {
    const auto* conflicts = graph.derived().conflicts().get(id);
    assert(conflicts);
    if (!conflicts) return {};

    const auto& reg = graph.regulations();
    std::unordered_map<StreamId, StreamParams> cache;
    auto paramsOf = [&](StreamId sid) -> const StreamParams& {
        auto it = cache.find(sid);
        if (it == cache.end())
            it = cache.emplace(sid, streamParams(graph, sid)).first;
        return it->second;
    };

    auto intergreen = [&](StreamId evacId, StreamId apprId,
                          const std::vector<ConflictPoint>& cps) {
        const auto& evac = paramsOf(evacId);
        const auto& appr = paramsOf(apprId);

        IntergreenDiagnostic::Base base{
            .v_evac = evac.v_evac,
            .v_appr = appr.v_appr,
            .l_evac = evac.l_evac,
            .t_y = evac.t_y,
            .t_min = reg.intergreenFloor(evac.kind, appr.kind),
            .t_apprAdd = reg.approachReactionMargin()};

        IntergreenDiagnostic best{.base = base};
        double tBest = std::numeric_limits<double>::lowest();
        for (const auto& cp : cps) {
            IntergreenDiagnostic diag{.base = base,
                                      .s_evac = cp.evacDistance,
                                      .s_appr = cp.apprDistance};
            if (double t = diag.t_raw(); t > tBest) {
                tBest = t;
                best = diag;
            }
        }
        int max = static_cast<int>(std::ceil(std::max(tBest, base.t_min)));
        return std::pair{max, best};
    };

    StreamIntergreenMaps result;
    conflicts->forEachPair([&](StreamId evacId, StreamId apprId,
                               const std::vector<ConflictPoint>& cpsFwd,
                               const std::vector<ConflictPoint>& cpsRev) {
        auto [maxFwd, diagFwd] = intergreen(evacId, apprId, cpsFwd);
        auto [maxRev, diagRev] = intergreen(apprId, evacId, cpsRev);
        result.max.set(evacId, apprId, maxFwd, maxRev);
        result.diag.set(evacId, apprId, diagFwd, diagRev);
    });
    return result;
}

//
// StreamPermittedMap
//
StreamMatrix<bool> streamPermittedMap(const Graph& graph, NodeId id) {
    const auto* conflicts = graph.derived().conflicts().get(id);
    assert(conflicts);
    if (!conflicts) return {};

    const auto& data = graph.data();
    const auto& reg = graph.regulations();

    StreamMatrix<bool> result;

    auto streamVector = [&](StreamId id) {
        return std::visit(
            overloaded{[&](MovementId mid) {
                           const auto* m = data.movements().get(mid);
                           const auto* e = data.entries().get(m->from());
                           return e->params().heading;
                       },
                       [&](CrosswalkId cid) {
                           const auto* cw = data.crosswalks().get(cid);
                           return cw->geometry().anchor.direction();
                       }},
            id);
    };

    conflicts->forEachPair([&](StreamId evacId, StreamId apprId,
                               const std::vector<ConflictPoint>&,
                               const std::vector<ConflictPoint>&) {
        // TODO: Expand to match 2311_2019: 8.3.2(a-e)
        bool fwd = reg.isParallel(streamVector(evacId), streamVector(apprId));
        bool rev = reg.isParallel(streamVector(apprId), streamVector(evacId));
        result.set(evacId, apprId, fwd, rev);
    });

    return result;
}

//
// StreamApproachMap
//
StreamMatrix<int> streamApproachMap(const Graph& graph, NodeId id) {
    const auto* conflicts = graph.derived().conflicts().get(id);
    assert(conflicts);
    if (!conflicts) return {};

    std::unordered_map<StreamId, StreamParams> cache;
    auto paramsOf = [&](StreamId sid) -> const StreamParams& {
        auto it = cache.find(sid);
        if (it == cache.end())
            it = cache.emplace(sid, streamParams(graph, sid)).first;
        return it->second;
    };

    // Travel time for the approaching stream to reach the shared point, taking
    // the nearest conflict point (soonest arrival) as the binding deadline.
    // Floored, so the deadline is conservative. -1 (inactive) when the pair
    // never meets, or when the approaching stream is a crosswalk: pedestrians
    // are already at the line, so they impose no deadline on anyone else.
    auto approach = [&](StreamId apprId,
                        const std::vector<ConflictPoint>& cps) -> int {
        if (std::holds_alternative<CrosswalkId>(apprId)) return -1;
        const auto& appr = paramsOf(apprId);
        double best = std::numeric_limits<double>::max();
        for (const auto& cp : cps)
            best = std::min(best, cp.apprDistance / appr.v_appr);
        if (best == std::numeric_limits<double>::max()) return -1;
        return static_cast<int>(std::floor(best));
    };

    StreamMatrix<int> result;
    conflicts->forEachPair([&](StreamId evacId, StreamId apprId,
                               const std::vector<ConflictPoint>& cpsFwd,
                               const std::vector<ConflictPoint>& cpsRev) {
        // Forward points have apprId approaching, so its travel time bounds
        // evacId (and symmetrically for the reverse).
        result.set(evacId, apprId, approach(apprId, cpsFwd),
                   approach(evacId, cpsRev));
    });
    return result;
}
}  // namespace graph_compute