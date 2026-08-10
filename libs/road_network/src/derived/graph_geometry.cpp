#include "road_network/graph/derived/graph_geometry.hpp"

#include <common/overloaded.hpp>

#include "road_network/graph/graph_data.hpp"

namespace graph_geometry {
namespace {
Position midpoint(Position a, Position b) {
    Position result;
    result.x = (a.x + b.x) / 2;
    result.y = (a.y + b.y) / 2;
    return result;
}
}  // namespace
std::vector<Position> lanePositions(const GraphData& data, EntryId id,
                                    bool withStopOffset) {
    if (!data.entries().alive(id)) return {};
    const auto* entry = data.entries().get(id);
    Position p = entry->params().position;
    Vector2 h = entry->params().heading;
    Vector2 perp = h.perpCw();
    double offsetPerp = 0.0;

    std::vector<Position> result;
    for (const auto& lane : entry->params().lanes) {
        Vector2 stopOffset =
            withStopOffset ? (-h * lane.stopLineOffset) : Vector2(0.0, 0.0);
        offsetPerp += lane.width * 0.5;
        result.push_back(p + perp * offsetPerp - stopOffset);
        offsetPerp += lane.width * 0.5;
    }
    return result;
}

std::vector<Polyline> entryStopBars(const GraphData& data, EntryId id,
                                    const std::vector<EntryLaneSpec>& lanes) {
    std::vector<Polyline> result;
    if (!data.entries().alive(id)) return result;
    const auto* entry = data.entries().get(id);
    Position p = entry->params().position;
    Vector2 h = entry->params().heading;
    Vector2 perp = h.perpCw();

    // Widths are additive: each lane spans [edge, edge + width] along perp.
    // The stop offset shifts the whole segment along the heading (same sign as
    // lanePositions, which subtracts a -h*offset).
    double edge = 0.0;
    for (const auto& lane : lanes) {
        double e0 = edge;
        edge += lane.width;
        double e1 = edge;
        Vector2 off = h * lane.stopLineOffset;
        Polyline poly;
        poly.setStart(p + perp * e0 + off);
        poly.addLine(p + perp * e1 + off);
        result.push_back(std::move(poly));
    }
    return result;
}

std::vector<Position> lanePositions(const GraphData& data, ExitId id) {
    if (!data.exits().alive(id)) return {};
    const auto* entry = data.exits().get(id);
    Position p = entry->params().position;
    Vector2 h = entry->params().heading;
    Vector2 perp = h.perpCw();
    double offsetPerp = 0.0;

    std::vector<Position> result;
    for (const auto& width : entry->params().lanes) {
        offsetPerp += width * 0.5;
        result.push_back(p + perp * offsetPerp);
        offsetPerp += width * 0.5;
    }
    return result;
}

std::vector<Polyline> exitLaneBars(const GraphData& data, ExitId id,
                                   const std::vector<double>& widths) {
    std::vector<Polyline> result;
    if (!data.exits().alive(id)) return result;
    const auto* exit = data.exits().get(id);
    Position p = exit->params().position;
    Vector2 h = exit->params().heading;
    Vector2 perp = h.perpCw();

    // Widths are additive: each lane spans [edge, edge + width] along perp.
    double edge = 0.0;
    for (const double width : widths) {
        double e0 = edge;
        edge += width;
        double e1 = edge;
        Polyline poly;
        poly.setStart(p + perp * e0);
        poly.addLine(p + perp * e1);
        result.push_back(std::move(poly));
    }
    return result;
}

std::optional<MovementAnchors> movementAnchors(const GraphData& data,
                                               MovementId id) {
    if (!data.movements().alive(id)) return std::nullopt;
    const auto* mov = data.movements().get(id);
    assert(mov->lanes().laneCount != 0);

    Vector2 eDir = data.entries().get(mov->from())->params().heading;
    Vector2 xDir = data.exits().get(mov->to())->params().heading;
    assert(!eDir.isZero() && !xDir.isZero());

    auto ePositionsNoOff = lanePositions(data, mov->from(), false);
    auto xPositions = lanePositions(data, mov->to());
    if (ePositionsNoOff.empty() || xPositions.empty()) return std::nullopt;

    auto eFront = static_cast<size_t>(mov->lanes().firstIdx);
    auto eLast = static_cast<size_t>(mov->lanes().lastIdx());
    return MovementAnchors{
        .eMid = midpoint(ePositionsNoOff[eFront], ePositionsNoOff[eLast]),
        .eDir = eDir,
        .xMid = midpoint(xPositions.front(), xPositions.back()),
        .xDir = xDir,
    };
}

std::vector<Polyline> movementPolylines(const GraphData& data, MovementId id) {
    if (!data.movements().alive(id)) return {};
    return movementPolylines(data, id,
                             data.movements().get(id)->params().pathSpec);
}

std::vector<Polyline> movementPolylines(const GraphData& data, MovementId id,
                                        const MovementPathSpec& spec) {
    std::vector<Polyline> result;
    auto anchors = movementAnchors(data, id);
    if (!anchors) return {};
    const auto* mov = data.movements().get(id);

    Vector2 eDir = anchors->eDir;
    Vector2 xDir = anchors->xDir;
    Position eMid = anchors->eMid;
    Position xMid = anchors->xMid;

    // Curve math anchors to the entry line (withStopOffset=false), so the knee
    // sits at eMid + eDir*dE regardless of the stop offset — matching
    // movementAnchors, which builds eMid the same way. The drawn path still
    // originates at the stop line: eStartPositions supplies that origin, and
    // because the stop line, entry line and knee are colinear along the heading
    // (eStart = ePos + eDir*stopLineOffset), the lead-in stop line → knee is a
    // single straight segment.
    auto ePositions =
        lanePositions(data, mov->from(), /*withStopOffset=*/false);
    auto eStartPositions =
        lanePositions(data, mov->from(), /*withStopOffset=*/true);
    auto xPositions = lanePositions(data, mov->to());

    size_t xi = 0;  // exit iterator
    for (const auto& count : mov->params().splitPerLane)
        xi += static_cast<size_t>(count);
    assert(xi == mov->lanes().laneCount);

    auto eFront = static_cast<size_t>(mov->lanes().firstIdx);
    size_t eBack = eFront + static_cast<size_t>(mov->lanes().laneCount);

    auto singlePolyline = [&](Position ePos, Position eStart, Position xPos) {
        auto lineImpl = [&] {
            Position eKnee = eMid + eDir * spec.dE;
            Position xKnee = xMid - xDir * spec.dX;

            Vector2 kneeDiff = eKnee - xKnee;
            Vector2 bridge =
                kneeDiff.isZero() ? eDir.perpCw() : kneeDiff.normalized();

            Vector2 eMitreDir = eDir + bridge;
            Vector2 xMitreDir = bridge + xDir;
            Line eMitre(eKnee,
                        eKnee + (eMitreDir.isZero() ? eDir.perpCw()
                                                    : eMitreDir.normalized()));
            Line xMitre(xKnee,
                        xKnee - (xMitreDir.isZero() ? xDir.perpCw()
                                                    : xMitreDir.normalized()));
            Position p1o, p2o;

            if (!eMitre.intersection({.p1 = ePos, .p2 = ePos + eDir}, p1o))
                p1o = ePos + (eDir * spec.dE);
            if (!xMitre.intersection({.p1 = xPos, .p2 = xPos - xDir}, p2o))
                p2o = xPos - (xDir * spec.dX);

            Polyline poly;
            poly.setStart(eStart);
            poly.addLine(p1o);
            poly.addLine(p2o);
            poly.addLine(xPos);
            return poly;
        };

        return std::visit(
            overloaded{
                [&](LineParams) -> Polyline { return lineImpl(); },
                [&](QuadBezierParams) -> Polyline {
                    Position p1 = ePos + eDir * spec.dE;
                    Position p2 = xPos - xDir * spec.dX;

                    // a) collinear entry/exit: tangent lines are parallel,
                    // no intersection
                    Position c;
                    if (!Line{.p1 = ePos, .p2 = ePos + eDir}.intersection(
                            {.p1 = xPos, .p2 = xPos + xDir}, c))
                        return lineImpl();

                    // b) control point is behind p1 or beyond p2: curve
                    // would reverse
                    if ((c - p1).dot(eDir) <= 0) return lineImpl();
                    if ((p2 - c).dot(xDir) <= 0) return lineImpl();

                    Polyline poly;
                    poly.setStart(eStart);
                    poly.addLine(p1);
                    poly.addQuad(c, p2);
                    poly.addLine(xPos);
                    return poly;
                },
                [&](CubicBezierParams params) -> Polyline {
                    Position eKnee = eMid + eDir * (spec.dE + params.cE);
                    Position xKnee = xMid - xDir * (spec.dX + params.cX);

                    Vector2 kneeDiff = eKnee - xKnee;
                    Vector2 bridge = kneeDiff.isZero() ? eDir.perpCw()
                                                       : kneeDiff.normalized();

                    Vector2 eMitreDir = eDir + bridge;
                    Vector2 xMitreDir = bridge + xDir;
                    Line eMitre(eKnee, eKnee + (eMitreDir.isZero()
                                                    ? eDir.perpCw()
                                                    : eMitreDir.normalized()));
                    Line xMitre(xKnee, xKnee - (xMitreDir.isZero()
                                                    ? xDir.perpCw()
                                                    : xMitreDir.normalized()));

                    Position p1o = ePos + eDir * spec.dE;
                    Position p2o = xPos - xDir * spec.dX;

                    // Intersect the mitres with the entry/exit tangent lines,
                    // taken from the heading directly. Deriving them from
                    // {ePos, p1o} collapses to a point when dE == 0 (likewise
                    // dX), which fails the solve and drops us to lineImpl.
                    Position c1, c2;
                    if (!eMitre.intersection({.p1 = ePos, .p2 = ePos + eDir},
                                             c1) ||
                        !xMitre.intersection({.p1 = xPos, .p2 = xPos - xDir},
                                             c2))
                        return lineImpl();

                    Polyline poly;
                    poly.setStart(eStart);
                    poly.addLine(p1o);
                    poly.addCubic(c1, c2, p2o);
                    poly.addLine(xPos);
                    return poly;
                }},
            spec.params);
    };

    auto splitPerLane = mov->params().splitPerLane;
    auto xLaneCount = data.exits().get(mov->to())->params().lanes.size();
    auto allotedSplits = 0;
    for (auto s : splitPerLane) allotedSplits += s;
    allotedSplits -= splitPerLane.back();
    splitPerLane.back() = xLaneCount - allotedSplits;

    xi = 0;
    // ei - entry iterator (global lane index; splitPerLane is
    // section-relative, sized laneCount — see setMovementSplitPerLane)
    for (size_t ei = eFront; ei < eBack; ++ei) {
        int splitCount = splitPerLane.at(ei - eFront);
        Position ePos = ePositions.at(ei);
        Position eStart = eStartPositions.at(ei);

        size_t xiStart = xi;
        xi += static_cast<size_t>(splitCount);

        for (size_t i = xiStart; i < xi; ++i) {
            Position xPos = xPositions.at(i);
            if (Polyline p = singlePolyline(ePos, eStart, xPos); !p.empty())
                result.push_back(p);
        }
    }
    return result;
}

}  // namespace graph_geometry