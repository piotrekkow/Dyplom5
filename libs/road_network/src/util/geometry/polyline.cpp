#include <cmath>
#include <optional>
#include <road_network/util/geometry/polyline.hpp>
#include <vector>

#include "geom_eps.hpp"
#include "road_network/util/overloaded.hpp"

namespace {

bool isQuadFlat(Position p1, Position p2, Position c, float tolerance) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;

    // Cross product to find distance from point 'c' to line 'p1-p2'
    float ux = c.x - p1.x;
    float uy = c.y - p1.y;
    float area = ux * dy - uy * dx;

    // Squared distance from c to line p1-p2
    float distSq = (area * area) / (dx * dx + dy * dy);

    return distSq <= (tolerance * tolerance);
}

bool isCubicFlat(Position p1, Position p2, Position c1, Position c2,
                 float tolerance) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float tolSq = tolerance * tolerance;
    float lineLenSq = dx * dx + dy * dy;

    // Handle the case where p1 and p2 are the same point (degenerated line)
    if (lineLenSq < geomEps) {
        float d1sq =
            (c1.x - p1.x) * (c1.x - p1.x) + (c1.y - p1.y) * (c1.y - p1.y);
        float d2sq =
            (c2.x - p1.x) * (c2.x - p1.x) + (c2.y - p1.y) * (c2.y - p1.y);
        return d1sq <= tolSq && d2sq <= tolSq;
    }

    // Distance for c1
    float area1 = (c1.x - p1.x) * dy - (c1.y - p1.y) * dx;
    if ((area1 * area1) / lineLenSq > tolSq) return false;

    // Distance for c2
    float area2 = (c2.x - p1.x) * dy - (c2.y - p1.y) * dx;
    if ((area2 * area2) / lineLenSq > tolSq) return false;

    return true;
}

void subdivideQuadraticBezier(Position p1, Position p2, Position c,
                              float tolerance, std::vector<Position>& out) {
    Position m0 = {.x = (p1.x + c.x) / 2.f, .y = (p1.y + c.y) / 2.f};
    Position m1 = {.x = (c.x + p2.x) / 2.f, .y = (c.y + p2.y) / 2.f};
    Position m2 = {.x = (m0.x + m1.x) / 2.f, .y = (m0.y + m1.y) / 2.f};

    if (isQuadFlat(p1, p2, c, tolerance)) {
        out.push_back(p2);  // Segment is flat enough
    } else {
        subdivideQuadraticBezier(p1, m2, m0, tolerance, out);
        subdivideQuadraticBezier(m2, p2, m1, tolerance, out);
    }
}

void subdivideCubicBezier(Position p1, Position p2, Position c1, Position c2,
                          float tolerance, std::vector<Position>& out) {
    Position m0 = {.x = (p1.x + c1.x) / 2, .y = (p1.y + c1.y) / 2};
    Position m1 = {.x = (c1.x + c2.x) / 2, .y = (c1.y + c2.y) / 2};
    Position m2 = {.x = (c2.x + p2.x) / 2, .y = (c2.y + p2.y) / 2};
    Position q0 = {.x = (m0.x + m1.x) / 2, .y = (m0.y + m1.y) / 2};
    Position q1 = {.x = (m1.x + m2.x) / 2, .y = (m1.y + m2.y) / 2};
    Position b = {.x = (q0.x + q1.x) / 2, .y = (q0.y + q1.y) / 2};

    if (isCubicFlat(p1, p2, c1, c2, tolerance)) {
        out.push_back(p2);
    } else {
        subdivideCubicBezier(p1, b, m0, q0, tolerance, out);
        subdivideCubicBezier(b, p2, q1, m2, tolerance, out);
    }
}

void tessellateInto(Position from, const Polyline::Segment& seg,
                    float tolerance, std::vector<Position>& out) {
    std::visit(
        overloaded{
            [&](const Polyline::LineSeg& s) { out.push_back(s.to); },
            [&](const Polyline::QuadSeg& s) {
                subdivideQuadraticBezier(from, s.to, s.ctrl, tolerance, out);
            },
            [&](const Polyline::CubicSeg& s) {
                subdivideCubicBezier(from, s.to, s.c1, s.c2, tolerance, out);
            },
        },
        seg);
}

bool segmentIntersectionProper(Position a1, Position a2, Position b1,
                               Position b2, Position& out) {
    Vector2 r = a2 - a1;
    Vector2 s = b2 - b1;

    float rxs = r.dx * s.dy - r.dy * s.dx;
    Vector2 qp = b1 - a1;

    if (std::abs(rxs) < geomEps) return false;  // parallel or collinear — skip

    float t = (qp.dx * s.dy - qp.dy * s.dx) / rxs;
    float u = (qp.dx * r.dy - qp.dy * r.dx) / rxs;

    if (t >= -geomEps && t < 1.f - geomEps && u >= -geomEps &&
        u < 1.f - geomEps) {
        out = a1 + r * t;
        return true;
    }

    return false;
}
}  // namespace

void Polyline::addLine(Position to) {
    segments_.emplace_back(LineSeg{to});
    recalculatePositions();
}

void Polyline::addQuad(Position ctrl, Position to) {
    segments_.emplace_back(QuadSeg{.ctrl = ctrl, .to = to});
    recalculatePositions();
}

void Polyline::addCubic(Position c1, Position c2, Position to) {
    segments_.emplace_back(CubicSeg{.c1 = c1, .c2 = c2, .to = to});
    recalculatePositions();
}

void Polyline::recalculatePositions() {
    positions_.clear();
    positions_.push_back(start_);

    for (const auto& seg : segments_) {
        Position from = positions_.back();
        std::visit(overloaded{
                       [&](const LineSeg& s) { positions_.push_back(s.to); },
                       [&](const QuadSeg& s) {
                           subdivideQuadraticBezier(from, s.to, s.ctrl,
                                                    tolerance_, positions_);
                       },
                       [&](const CubicSeg& s) {
                           subdivideCubicBezier(from, s.to, s.c1, s.c2,
                                                tolerance_, positions_);
                       },
                   },
                   seg);
    }
}

std::optional<Polyline::Intersection> Polyline::firstIntersection(
    const Line& other) const {
    if (segments_.empty()) return std::nullopt;

    float accA = 0.f;
    Position from = start_;

    for (const auto& seg : segments_) {
        std::vector<Position> pts = {from};
        tessellateInto(from, seg, tolerance_, pts);

        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            Position hit;
            if (segmentIntersectionProper(pts[i], pts[i + 1], other.p1,
                                          other.p2, hit)) {
                return Intersection{hit, accA + Line(pts[i], hit).length(),
                                    Line(other.p1, hit).length()};
            }
            accA += Line(pts[i], pts[i + 1]).length();
        }

        from = pts.back();
    }
    return std::nullopt;
}

std::optional<Polyline::Intersection> Polyline::lastIntersection(
    const Polyline& other) const {
    if (segments_.empty() || other.segments_.empty()) return std::nullopt;

    // Tessellate both polylines from their segments into flat point lists.
    std::vector<Position> pa = {start_};
    for (const auto& seg : segments_) {
        Position from = pa.back();
        tessellateInto(from, seg, tolerance_, pa);
    }

    std::vector<Position> pb = {other.start_};
    for (const auto& seg : other.segments_) {
        Position from = pb.back();
        tessellateInto(from, seg, other.tolerance_, pb);
    }

    if (pa.size() < 2 || pb.size() < 2) return std::nullopt;

    std::optional<Intersection> lastHit;
    float accA = 0.f;

    for (size_t i = 0; i + 1 < pa.size(); ++i) {
        float accB = 0.f;
        for (size_t j = 0; j + 1 < pb.size(); ++j) {
            Position hit;
            if (segmentIntersectionProper(pa[i], pa[i + 1], pb[j], pb[j + 1],
                                          hit)) {
                float distA = accA + Line(pa[i], hit).length();
                float distB = accB + Line(pb[j], hit).length();
                if (!lastHit || distA > lastHit->distanceA) {
                    lastHit = Intersection{
                        .point = hit, .distanceA = distA, .distanceB = distB};
                }
            }
            accB += Line(pb[j], pb[j + 1]).length();
        }
        accA += Line(pa[i], pa[i + 1]).length();
    }

    return lastHit;
}

[[nodiscard]] float Polyline::length() const {
    if (positions_.size() < 2) {
        return 0.f;
    }

    float total = 0.f;

    for (size_t i = 1; i < positions_.size(); ++i) {
        total += (positions_[i] - positions_[i - 1]).length();
    }

    return total;
}