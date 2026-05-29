#pragma once

#include <utility>

#include "road_network/util/ids.hpp"
#include "stream_type.hpp"

struct LineParams {};
struct QuadBezierParams {
};  // control point is derived from entry and exit line intersection
struct CubicBezierParams {
    float cE;  // first control point offset along entry line
    float cX;  // second control point offset along exit line
};
using MovementGeometryParams =
    std::variant<LineParams, QuadBezierParams, CubicBezierParams>;
struct MovementPathSpec {
    float dE = 0.f;  // offset along entry line
    float dX = 0.f;  // offset along exit line
    MovementGeometryParams params = LineParams{};
};

// one movement per entry-exit pair
class Movement {
   public:
    Movement(EntryId from, ExitId to, std::vector<EntryLaneId> lanes,
             std::vector<int> splitPerLane)
        : from_(from),
          to_(to),
          lanes_(std::move(lanes)),
          splitPerLane_(std::move(splitPerLane)) {}

    [[nodiscard]] EntryId from() const { return from_; }
    [[nodiscard]] ExitId to() const { return to_; }
    [[nodiscard]] const MovementPathSpec& pathSpec() const { return pathSpec_; }
    [[nodiscard]] const std::vector<EntryLaneId>& lanes() const {
        return lanes_;
    }
    [[nodiscard]] const std::vector<int>& splitPerLane() const {
        return splitPerLane_;
    }
    [[nodiscard]] MovementType type() const { return type_; }

    void setPathSpec(MovementPathSpec spec) { pathSpec_ = spec; }
    void setSplits(std::vector<int> splits) {
        splitPerLane_ = std::move(splits);
    }
    void setType(MovementType t) { type_ = t; }

   private:
    EntryId from_;
    ExitId to_;
    std::vector<EntryLaneId> lanes_;
    std::vector<int> splitPerLane_;  // into how many exit lanes does a given
    // entry lane split
    MovementPathSpec pathSpec_ = {
        .dE = 5.f, .dX = 5.f, .params = QuadBezierParams{}};
    MovementType type_ = MovementType::VEHICLE;
};