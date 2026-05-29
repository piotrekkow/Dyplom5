#pragma once

#include "road_network/util/geometry/position.hpp"
#include "road_network/util/ids.hpp"

class Exit {
   public:
    Exit(NodeId at, Position position, Vector2 heading)
        : from_(at), position_(position), heading_(heading.normalized()) {}

    [[nodiscard]] Position position() const { return position_; }
    [[nodiscard]] Vector2 heading() const { return heading_; }
    [[nodiscard]] float speedLimit() const { return speedLimit_; }
    [[nodiscard]] NodeId at() const { return from_; }
    [[nodiscard]] const std::vector<ExitLaneId>& lanes() const {
        return lanes_;
    }

    void setPosition(Position position) { position_ = position; }
    void setHeading(Vector2 heading) { heading_ = heading.normalized(); }
    void setSpeedLimit(float speedLimit) { speedLimit_ = speedLimit; }

    std::vector<ExitLaneId>& lanes() { return lanes_; }

   private:
    NodeId from_;
    // std::optional<EdgeId> edge_ = std::nullopt;

    Position position_;
    Vector2 heading_;
    float speedLimit_ = 13.88f;

    std::vector<ExitLaneId> lanes_;  // encodes order of lanes
};