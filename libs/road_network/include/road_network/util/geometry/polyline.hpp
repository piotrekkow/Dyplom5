#pragma once
#include <optional>
#include <variant>
#include <vector>

#include "line.hpp"
#include "position.hpp"

class Polyline {
   public:
    struct LineSeg {
        Position to;
    };
    struct QuadSeg {
        Position ctrl, to;
    };
    struct CubicSeg {
        Position c1, c2, to;
    };
    using Segment = std::variant<LineSeg, QuadSeg, CubicSeg>;

    struct Intersection {
        Position point;
        float distanceA;
        float distanceB;
    };

    Polyline() = default;

    void setStart(Position start) {
        start_ = start;
        recalculatePositions();
    }

    void addLine(Position to);
    void addQuad(Position ctrl, Position to);
    void addCubic(Position c1, Position c2, Position to);

    void reset() {
        segments_.clear();
        recalculatePositions();
    }

    [[nodiscard]] bool empty() const { return segments_.empty(); }

    [[nodiscard]] const std::vector<Position>& positions() const {
        return positions_;
    }
    [[nodiscard]] std::optional<Intersection> firstIntersection(
        const Line& other) const;
    [[nodiscard]] std::optional<Intersection> lastIntersection(
        const Polyline& other) const;

    [[nodiscard]] Position start() const { return start_; }
    [[nodiscard]] const std::vector<Segment>& segments() const {
        return segments_;
    }
    [[nodiscard]] float length() const;

   private:
    Position start_ = {.x = 0.f, .y = 0.f};
    std::vector<Segment> segments_;  // absolute
    std::vector<Position>
        positions_;  // absolute, tessellated, for intersection
    float tolerance_ = 0.05f;

    void recalculatePositions();
};