#include "graph_compute.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "road_network/graph.hpp"
#include "road_network/graph/derived/graph_geometry.hpp"
#include "road_network/graph/derived/turn_type.hpp"
#include "road_network/graph/graph_data.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/overloaded.hpp"

namespace graph_compute {
namespace {

//
//  ConflictMap
//
std::unordered_map<MovementId, std::vector<graph_geometry::MovementPolyline>>
movementGeometries(const GraphData& data, NodeId id) {
    std::unordered_map<MovementId,
                       std::vector<graph_geometry::MovementPolyline>>
        result;
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

std::unordered_map<CrosswalkId, Crosswalk::Lines> crosswalkGeometries(
    const GraphData& data, NodeId id) {
    std::unordered_map<CrosswalkId, Crosswalk::Lines> result;
    auto* node = data.nodes().get(id);
    if (!node) return result;

    for (auto csId : node->crosswalkSeries()) {
        for (auto cId : data.crosswalkSeries().get(csId)->crosswalks()) {
            result[cId] = data.crosswalks().get(cId)->lines();
        }
    }

    return result;
}

void conflictsMovMov(
    ConflictMap& out, MovementId evacId, MovementId approachId,
    const std::vector<graph_geometry::MovementPolyline>& evacPaths,
    const std::vector<graph_geometry::MovementPolyline>& approachPaths) {
    for (const auto& evacPath : evacPaths) {
        for (const auto& approachPath : approachPaths) {
            auto pathIntersection =
                evacPath.poly.lastIntersection(approachPath.poly);
            if (!pathIntersection.has_value()) continue;
            auto pi = pathIntersection.value();

            out[evacId][approachId].push_back(
                {.position = pi.point,
                 .evacDistance = pi.distanceA - evacPath.stopOffset,
                 .approachDistance = pi.distanceB - approachPath.stopOffset});
        }
    }
}

void conflictsMovCross(
    ConflictMap& out, MovementId evacId, CrosswalkId approachId,
    const std::vector<graph_geometry::MovementPolyline>& evacPaths,
    const Crosswalk::Lines& apprCrosswalk) {
    for (const auto& evacPath : evacPaths) {
        auto intersection1 = evacPath.poly.firstIntersection(apprCrosswalk.l1);
        if (!intersection1) continue;
        auto intersection2 = evacPath.poly.firstIntersection(apprCrosswalk.l2);

        // TODO: IntergreenDiagnostic { MovementId, CrosswalkId };
        // struct IntergreenResult { StreamIntergreenMap;
        // std::vector<IntergreenDiagnostic> };
        assert(intersection2 &&
               "movement crosses crosswalk entry but not exit - malformed "
               "geometry");

        auto& furthest = intersection1->distanceA > intersection2->distanceA
                             ? intersection1
                             : intersection2;

        out[evacId][approachId].push_back({
            .position = furthest->point,
            .evacDistance = furthest->distanceA - evacPath.stopOffset,
            .approachDistance = 0.f,
        });
    }
}
void conflictsCrossMov(
    ConflictMap& out, CrosswalkId evacId, MovementId approachId,
    const Crosswalk::Lines& evacCrosswalk,
    const std::vector<graph_geometry::MovementPolyline>& approachPaths) {
    for (const auto& approachPath : approachPaths) {
        auto intersection1 =
            approachPath.poly.firstIntersection(evacCrosswalk.l1);
        if (!intersection1) continue;
        auto intersection2 =
            approachPath.poly.firstIntersection(evacCrosswalk.l2);

        assert(intersection2 &&
               "Invalid intersection geometry where movement doesn't "
               "cross both ends of a crosswalk");

        auto& closest = intersection1->distanceA < intersection2->distanceA
                            ? intersection1
                            : intersection2;
        float maxCrosswalkLineLength =
            std::max(evacCrosswalk.l1.length(), evacCrosswalk.split.length());

        out[evacId][approachId].push_back(
            {.position = closest->point,
             .evacDistance = maxCrosswalkLineLength,
             .approachDistance = closest->distanceB - approachPath.stopOffset});
    }
}

//
//  IntergreenStreamMap
//
struct StreamParams {
    float t_y;    // yellow time
    float l_p;    // length extension for evac
    float v_e;    // evacuation speed
    float v_d;    // approach speed
    float t_min;  // min intergreen time
};

StreamParams streamParams(const GraphData& data, StreamId id) {
    return std::visit(
        overloaded{[&](MovementId mid) -> StreamParams {
                       auto* m = data.movements().get(mid);

                       float speedLimit =
                           std::min(data.entries().get(m->from())->speedLimit(),
                                    data.exits().get(m->to())->speedLimit());

                       float travelSpeed;
                       switch (graph_geometry::classifyTurn(data, mid)) {
                           case TurnType::UTURN:
                               travelSpeed = 8.33f;
                               break;
                           case TurnType::LEFT:
                               travelSpeed = 11.11f;
                               break;
                           case TurnType::RIGHT:
                               travelSpeed = 8.33f;
                               break;
                           case TurnType::THROUGH:
                               travelSpeed = speedLimit;
                               break;
                       }

                       switch (m->type()) {
                           default:
                           case MovementType::VEHICLE:
                               return {.t_y = 3.f,
                                       .l_p = 10.f,
                                       .v_e = std::min(
                                           {travelSpeed, speedLimit, 14.f}),
                                       .v_d = std::max(speedLimit, 16.7f),
                                       .t_min = 1.f};
                               // case MovementType::BUS:
                               //     return {.t_y = 3.f,
                               //             .l_p = 14.f,
                               //             .v_e = std::min(speedLimit, 10.f),
                               //             .v_d = std::min(speedLimit, 10.f),
                               //             .t_min = 1.f};
                               // case MovementType::TRAM:
                               //     return {.t_y = 3.f,
                               //             .l_p = 13.5f,  // per wagon —
                               //             caller multiplies .v_e =
                               //             std::min(speedLimit, 10.f), .v_d =
                               //             std::min(speedLimit, 10.f), .t_min
                               //             = 1.f};
                       }
                   },
                   [&](CrosswalkId cid) -> StreamParams {
                       auto* cw = data.crosswalks().get(cid);
                       bool isCyclist = cw->type() == CrosswalkType::CYCLIST;
                       return isCyclist ? StreamParams{.t_y = 0.f,
                                                       .l_p = 0.f,
                                                       .v_e = 4.2f,
                                                       .v_d = 4.2f,
                                                       .t_min = 4.0f}
                                        : StreamParams{.t_y = 0.f,
                                                       .l_p = 0.f,
                                                       .v_e = 1.4f,
                                                       .v_d = 1.4f,
                                                       .t_min = 4.0f};
                   }},
        id);
}

}  // namespace

ConflictMap conflictMap(const GraphData& data, NodeId id) {
    ConflictMap cm;

    auto movGeometries = movementGeometries(data, id);
    auto crossGeometries = crosswalkGeometries(data, id);

    /// MOVEMENT->MOVEMENT Conflicts
    for (const auto& [evacId, evacGeo] : movGeometries) {
        auto* evac = data.movements().get(evacId);
        if (!evac) continue;
        for (const auto& [approachId, approachGeo] : movGeometries) {
            auto* approach = data.movements().get(approachId);
            if (!approach) continue;
            if (evac->from() == approach->from()) continue;
            conflictsMovMov(cm, evacId, approachId, evacGeo, approachGeo);
        }
    }

    /// MOVEMENT->CROSSING Conflicts
    for (const auto& [evacId, evacGeo] : movGeometries) {
        for (const auto& [approachId, approachGeo] : crossGeometries) {
            conflictsMovCross(cm, evacId, approachId, evacGeo, approachGeo);
        }
    }

    /// CROSSING->MOVEMENT Conflicts
    for (const auto& [evacId, evacGeo] : crossGeometries) {
        for (const auto& [approachId, approachGeo] : movGeometries) {
            conflictsCrossMov(cm, evacId, approachId, evacGeo, approachGeo);
        }
    }

    return cm;
}

StreamIntergreenMap streamIntergreenMap(const Graph& graph, NodeId id) {
    StreamIntergreenMap result;
    const auto* conflicts = graph.derived().conflict().get(id);
    assert(conflicts && "Conflicts missing for StreamIntergreenMap");
    if (!conflicts) return result;

    for (const auto& [evacuatingId, row] : *conflicts) {
        auto evac = streamParams(graph.data(), evacuatingId);
        for (const auto& [approachingId, conflictPoints] : row) {
            auto appr = streamParams(graph.data(), approachingId);
            float t_last = std::numeric_limits<float>::min();
            StreamIntergreenEntry ig;
            for (const auto& cp : conflictPoints) {
                float t_e = (cp.evacDistance + evac.l_p) / evac.v_e;
                float t_d = cp.approachDistance / appr.v_d;
                float t = evac.t_y + t_e - t_d;
                float t_max = std::ceil(std::max(t, evac.t_min));
                if (t > t_last) {
                    ig = {.t_max = static_cast<int>(t_max),
                          .diag = {.s_evac = cp.evacDistance,
                                   .s_appr = cp.approachDistance,
                                   .v_evac = evac.v_e,
                                   .v_appr = appr.v_d,
                                   .l_evac = evac.l_p,
                                   .t_y = evac.t_y,
                                   .t_min = evac.t_min,
                                   .t_evac = t_e,
                                   .t_appr = t_d,
                                   .t_raw = t}};
                    t_last = t;
                }
            }
            result[evacuatingId][approachingId] = ig;
        }
    }
    return result;
}

StreamPermittedMap streamPermittedMap(const Graph& graph, NodeId id,
                                      StreamPermittedCtx ctx) {
    StreamPermittedMap result;
    const auto& data = graph.data();
    const auto* conflicts = graph.derived().conflict().get(id);
    assert(conflicts && "Conflicts missing for StreamPermittedMap");
    if (!conflicts) return result;

    auto isOpposing = [&](EntryId a, EntryId b) -> bool {
        const auto* ea = data.entries().get(a);
        const auto* eb = data.entries().get(b);
        if (!ea || !eb) return false;
        return ea->heading().dot(eb->heading()) < ctx.kOpposingThreshold;
    };

    auto isParallelToCrosswalk = [&](EntryId entryId,
                                     CrosswalkId cwId) -> bool {
        const auto* entry = data.entries().get(entryId);
        const auto* cw = data.crosswalks().get(cwId);
        if (!entry || !cw) return false;
        Vector2 crossDir = cw->params().line.direction().normalized();
        return std::abs(entry->heading().dot(crossDir)) >
               ctx.kParallelThreshold;
    };

    auto isPermitted = [&](StreamId evacId, StreamId apprId) -> bool {
        return std::visit(
            overloaded{
                [&](MovementId evacMid) {
                    return std::visit(
                        overloaded{
                            [&](MovementId apprMid) -> bool {
                                if (graph_geometry::isOpposingEntry(
                                        data, evacMid, apprMid)) {
                                    // max turn lane for perm movement rule
                                    auto tt = graph_geometry::classifyTurn(
                                        data, evacMid);
                                    if (tt == TurnType::UTURN ||
                                        tt == TurnType::LEFT) {
                                        const auto* em =
                                            data.movements().get(evacMid);
                                        return em->lanes().size() <=
                                               ctx.permMaxTurnLaneMovementLaneCount;
                                    }
                                    return true;
                                }
                                return false;
                            },
                            [&](CrosswalkId apprCid) -> bool {
                                const auto* em = data.movements().get(evacMid);
                                if (!em) return false;
                                return isParallelToCrosswalk(em->from(),
                                                             apprCid);
                            }},
                        apprId);
                },
                [&](CrosswalkId evacCid) {
                    return std::visit(
                        overloaded{[&](MovementId apprMid) -> bool {
                                       const auto* am =
                                           data.movements().get(apprMid);
                                       if (!am) return false;
                                       return isParallelToCrosswalk(am->from(),
                                                                    evacCid);
                                   },
                                   [&](CrosswalkId) -> bool { return false; }},
                        apprId);
                }},
            evacId);
    };

    for (const auto& [evacuatingId, row] : *conflicts) {
        for (const auto& [approachingId, conflictPoints] : row) {
            if (conflictPoints.empty()) continue;

            bool permitted = isPermitted(evacuatingId, approachingId);

            if (permitted) {
                result[evacuatingId].insert(approachingId);
                result[approachingId].insert(evacuatingId);
            }
        }
    }
    return result;
}
}  // namespace graph_compute