#include "road_network/graph/derived/graph_geometry.hpp"

#include <cassert>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numbers>
#include <unordered_map>

#include "road_network/graph/graph_data.hpp"
#include "road_network/util/geometry/polyline.hpp"
#include "road_network/util/geometry/position.hpp"
#include "road_network/util/geometry/vector2.hpp"
#include "road_network/util/overloaded.hpp"

namespace {
// const — for read-only access
inline const Entry* adptGet(const GraphData& g, EntryId id) {
    return g.entries().get(id);
}
inline const Exit* adptGet(const GraphData& g, ExitId id) {
    return g.exits().get(id);
}

inline Position adptPos(const Entry& a) { return a.position(); }
inline Position adptPos(const Exit& a) { return a.position(); }

inline Vector2 adptHeading(const Entry& a) { return a.heading(); }
inline Vector2 adptHeading(const Exit& a) {
    return a.heading();
}  // exits face outward

template <typename F>
void forEachLane(const GraphData& g, const Entry& e, F&& f) {
    for (auto id : e.lanes())
        if (auto* lane = g.entryLanes().get(id)) f(lane->width(), id);
}

template <typename F>
void forEachLane(const GraphData& g, const Exit& e, F&& f) {
    for (auto id : e.lanes())
        if (auto* lane = g.exitLanes().get(id)) f(lane->width(), id);
}

Position midpoint(std::vector<Position> positions) {
    assert(!positions.empty());
    Position p = positions.front();
    if (positions.size() > 1) {
        Vector2 lateral = (positions.back() - positions.front()) / 2.f;
        p = p + lateral;
    }
    return p;
}

struct MovementPolylineCtx {
    Position eMidpoint;
    Position xMidpoint;
    Vector2 eDir;
    Vector2 xDir;
};

Polyline lineImpl(const Movement* m, Position ePos, Position xPos,
                  MovementPolylineCtx ctx) {
    Position eKnee = ctx.eMidpoint + ctx.eDir * m->pathSpec().dE;
    Position xKnee = ctx.xMidpoint - ctx.xDir * m->pathSpec().dX;

    Vector2 kneeDiff = eKnee - xKnee;
    Vector2 bridge =
        kneeDiff.isZero() ? ctx.eDir.perpCw() : kneeDiff.normalized();

    Vector2 eMitreDir = ctx.eDir + bridge;
    Vector2 xMitreDir = bridge + ctx.xDir;
    Line eMitre(eKnee, eKnee + (eMitreDir.isZero() ? ctx.eDir.perpCw()
                                                   : eMitreDir.normalized()));
    Line xMitre(xKnee, xKnee - (xMitreDir.isZero() ? ctx.xDir.perpCw()
                                                   : xMitreDir.normalized()));
    Position p1o, p2o;

    if (!eMitre.intersection({ePos, ePos + ctx.eDir}, p1o))
        p1o = ePos + (ctx.eDir * m->pathSpec().dE);
    if (!xMitre.intersection({xPos, xPos - ctx.xDir}, p2o))
        p2o = xPos - (ctx.xDir * m->pathSpec().dX);

    Polyline poly;
    poly.setStart(ePos);
    poly.addLine(p1o);
    poly.addLine(p2o);
    poly.addLine(xPos);
    return poly;
}

}  // namespace

namespace graph_geometry {
std::vector<Position> lanePositions(const GraphData& data, AdapterId at) {
    return std::visit(
        [&](auto id) -> std::vector<Position> {
            auto* adpt = adptGet(data, id);
            if (!adpt) return {};
            auto heading = adptHeading(*adpt);
            auto pos = adptPos(*adpt);

            Vector2 perp = heading.perpCw();
            float offset = 0.f;

            std::vector<Position> result;
            forEachLane(data, *adpt, [&](float width, auto) {
                offset += width * 0.5f;
                result.push_back(pos + perp * offset);
                offset += width * 0.5f;
            });
            return result;
        },
        at);
}

Polyline singlePolyline(const Movement* m, Position ePos, Position xPos,
                        MovementPolylineCtx ctx) {
    return std::visit(
        overloaded{
            [&](LineParams) -> Polyline {
                return lineImpl(m, ePos, xPos, ctx);
            },
            [&](QuadBezierParams) -> Polyline {
                Position p1 = ePos + ctx.eDir * m->pathSpec().dE;
                Position p2 = xPos - ctx.xDir * m->pathSpec().dX;

                // a) collinear entry/exit: tangent lines are parallel, no
                // intersection
                Position c;
                if (!Line{ePos, ePos + ctx.eDir}.intersection(
                        {xPos, xPos + ctx.xDir}, c))
                    return lineImpl(m, ePos, xPos, ctx);

                // b) control point is behind p1 or beyond p2: curve would
                // reverse
                if ((c - p1).dot(ctx.eDir) <= 0)
                    return lineImpl(m, ePos, xPos, ctx);
                if ((p2 - c).dot(ctx.xDir) <= 0)
                    return lineImpl(m, ePos, xPos, ctx);

                Polyline poly;
                poly.setStart(ePos);
                poly.addLine(p1);
                poly.addQuad(c, p2);
                poly.addLine(xPos);
                return poly;
            },
            [&](CubicBezierParams params) -> Polyline {
                Position eKnee =
                    ctx.eMidpoint + ctx.eDir * (m->pathSpec().dE + params.cE);
                Position xKnee =
                    ctx.xMidpoint - ctx.xDir * (m->pathSpec().dX + params.cX);

                Vector2 kneeDiff = eKnee - xKnee;
                Vector2 bridge = kneeDiff.isZero() ? ctx.eDir.perpCw()
                                                   : kneeDiff.normalized();

                Vector2 eMitreDir = ctx.eDir + bridge;
                Vector2 xMitreDir = bridge + ctx.xDir;
                Line eMitre(eKnee, eKnee + (eMitreDir.isZero()
                                                ? ctx.eDir.perpCw()
                                                : eMitreDir.normalized()));
                Line xMitre(xKnee, xKnee - (xMitreDir.isZero()
                                                ? ctx.xDir.perpCw()
                                                : xMitreDir.normalized()));

                Position p1o = ePos + ctx.eDir * m->pathSpec().dE;
                Position p2o = xPos - ctx.xDir * m->pathSpec().dX;

                Position c1, c2;
                if (!eMitre.intersection({ePos, p1o}, c1) ||
                    !xMitre.intersection({xPos, p2o}, c2))
                    return lineImpl(m, ePos, xPos, ctx);

                Polyline poly;
                poly.setStart(ePos);
                poly.addLine(p1o);
                poly.addCubic(c1, c2, p2o);
                poly.addLine(xPos);
                return poly;
            }},
        m->pathSpec().params);
}

std::vector<MovementPolyline> movementPolylines(const GraphData& data,
                                                MovementId id) {
    std::vector<MovementPolyline> result;
    const auto* m = data.movements().get(id);
    if (!m || m->lanes().empty()) {
        std::cerr << "Cannot create geometry for movement (" << id.index()
                  << ") with no origins.\n";
        return {};
    }
    auto eDir = data.entries().get(m->from())->heading();
    auto xDir = data.exits().get(m->to())->heading();
    if (eDir.isZero()) {
        std::cerr << "Cannot create geometry for movement (" << id.index()
                  << ") with zero entry vector.\n";
        return {};
    }
    if (xDir.isZero()) {
        std::cerr << "Cannot create geometry for movement (" << id.index()
                  << ") with zero exit vector.\n";
        return {};
    }

    // Build entry lane ID → position map (preserves lateral ordering)
    std::unordered_map<EntryLaneId, Position> entryLanePosMap;
    {
        const auto* entry = data.entries().get(m->from());
        auto pos = entry->position();
        Vector2 perp = entry->heading().perpCw();
        float offset = 0.f;
        forEachLane(data, *entry, [&](float width, EntryLaneId lid) {
            offset += width * 0.5f;
            entryLanePosMap[lid] = pos + perp * offset;
            offset += width * 0.5f;
        });
    }

    auto exitPositions = lanePositions(data, m->to());

    size_t xi = 0;  // exit iterator

    // check exit lane count equal to entry lane split sum
    for (const auto& count : m->splitPerLane())
        xi += static_cast<size_t>(count);
    if (xi != data.exits().get(m->to())->lanes().size()) return {};

    // collect origin positions for eMidpoint
    std::vector<Position> originPositions;
    for (const auto& lId : m->lanes()) {
        auto it = entryLanePosMap.find(lId);
        if (it == entryLanePosMap.end()) return {};
        originPositions.push_back(it->second);
    }

    // construct entry/exit context
    MovementPolylineCtx ctx{.eMidpoint = midpoint(originPositions),
                            .xMidpoint = midpoint(exitPositions),
                            .eDir = eDir,
                            .xDir = xDir};

    xi = 0;
    // oi - origin iterator
    for (size_t oi = 0; oi < m->lanes().size(); ++oi) {
        int splitCount = m->splitPerLane().at(oi);
        EntryLaneId lId = m->lanes()[oi];
        auto ePos = entryLanePosMap.at(lId);
        const auto* lane = data.entryLanes().get(lId);
        assert(lane);
        if (!lane) return {};
        auto eStopOffset = lane->stopLineOffset();

        size_t xiStart = xi;
        xi += static_cast<size_t>(splitCount);

        for (size_t i = xiStart; i < xi; ++i) {
            auto xPos = exitPositions[i];
            if (auto p = singlePolyline(m, ePos, xPos, ctx); !p.empty())
                result.push_back({p, eStopOffset});
        }
    }
    return result;
}

float classifyAngle(const GraphData& data, EntryId entry, ExitId exit) {
    const auto* e = data.entries().get(entry);
    const auto* x = data.exits().get(exit);
    auto ev = e->heading();
    auto mv = x->position() - e->position();
    auto xv = x->heading();
    return ev.angleTo(mv) + mv.angleTo(xv);
}

float classifyAngle(const GraphData& data, MovementId id) {
    const auto* m = data.movements().get(id);
    assert(m);
    if (!m) return 0.f;
    return classifyAngle(data, m->from(), m->to());
}

TurnType classifyTurn(const GraphData& data, MovementId id) {
    float angle = classifyAngle(data, id);
    // angle ∈ [-π/4,  π/4] → THROUGH
    // angle ∈ ( π/4, 3π/4] → LEFT
    // angle ∈ (3π/4,   +∞) → UTURN
    // angle ∈ (  -∞, -π/4) → RIGHT
    constexpr float a = std::numbers::pi_v<float> / 4;
    constexpr float b = 3 * std::numbers::pi_v<float> / 4;

    if (angle < -a) return TurnType::RIGHT;
    if (angle <= a) return TurnType::THROUGH;
    if (angle <= b) return TurnType::LEFT;

    return TurnType::UTURN;
}

bool isValidMovementOrder(const GraphData& data, EntryId id) {
    const auto* e = data.entries().get(id);
    assert(e);
    if (!e) return false;
    float last = std::numeric_limits<float>::max();
    for (auto mId : e->movements()) {
        auto angle = classifyAngle(data, mId);
        if (angle > last) return false;
        last = angle;
    }
    return true;
}

std::optional<EntryId> findOpposingEntry(const GraphData& data, EntryId id,
                                         float opposingThreshold) {
    const auto* e = data.entries().get(id);
    assert(e);

    const auto* node = data.nodes().get(e->at());
    std::optional<EntryId> result = std::nullopt;

    for (auto otherId : node->entries()) {
        if (otherId == id) continue;
        const auto* other = data.entries().get(otherId);
        if (!other) continue;

        float angle = std::abs(e->heading().angleTo(other->heading()));
        if (angle >= opposingThreshold) {
            opposingThreshold = angle;
            result = otherId;
        }
    }

    return result;
}

bool isOpposingEntry(const GraphData& data, MovementId id, MovementId candidate,
                     float opposingThreshold) {
    auto m = data.movements().get(id);
    auto other = data.movements().get(candidate);
    assert(m && other);

    auto opp = findOpposingEntry(data, m->from(), opposingThreshold);
    return (opp && *opp == other->from());
}
}  // namespace graph_geometry