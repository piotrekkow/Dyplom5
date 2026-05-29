#pragma once

#include "road_network/util/geometry/position.hpp"
#include "road_network/util/ids.hpp"

class Entry {
   public:
    Entry(NodeId at, Position position, Vector2 heading)
        : to_(at), position_(position), heading_(heading.normalized()) {}

    [[nodiscard]] Position position() const { return position_; }
    [[nodiscard]] Vector2 heading() const { return heading_; }
    [[nodiscard]] float speedLimit() const { return speedLimit_; }
    [[nodiscard]] NodeId at() const { return to_; }
    [[nodiscard]] const std::vector<EntryLaneId>& lanes() const {
        return lanes_;
    }
    [[nodiscard]] const std::vector<MovementId>& movements() const {
        return movements_;
    }
    [[nodiscard]] const std::vector<ClusterId>& clusters() const {
        return clusters_;
    }

    void setPosition(Position position) { position_ = position; }
    void setHeading(Vector2 heading) { heading_ = heading.normalized(); }
    void setSpeedLimit(float speedLimit) { speedLimit_ = speedLimit; }
    void setClusters(std::vector<ClusterId> clusters) {
        clusters_ = std::move(clusters);
    }
    void clearClusters() { clusters_.clear(); }

    std::vector<MovementId>& movements() { return movements_; }
    std::vector<EntryLaneId>& lanes() { return lanes_; }
    std::vector<ClusterId>& clusters() { return clusters_; }

   private:
    // std::optional<EdgeId> edge_ = std::nullopt;
    NodeId to_;

    Position position_;
    Vector2 heading_;
    float speedLimit_ = 13.88f;

    std::vector<EntryLaneId> lanes_;     // encodes order of lanes
    std::vector<MovementId> movements_;  // encodes order of movements
    std::vector<ClusterId> clusters_;
};