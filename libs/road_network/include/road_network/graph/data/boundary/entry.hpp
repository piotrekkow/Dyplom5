#pragma once

#include <geometry/position.hpp>

#include "entry_lane_spec.hpp"
#include "road_network/util/ids.hpp"

class Entry {
   public:
    struct Params {
        Position position;
        Vector2 heading;
        double speedLimit = 50 / 3.6;  // m/s
        std::vector<EntryLaneSpec> lanes = {
            {.width = 3.5, .stopLineOffset = 0.0}};
    };

    Entry(NodeId at, Position position, Vector2 heading)
        : at_(at), params_(position, heading) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] const std::vector<MovementId>& movements() const {
        return movements_;
    }
    [[nodiscard]] const std::vector<ClusterId>& clusters() const {
        return clusters_;
    }
    [[nodiscard]] const Params& params() const { return params_; }

    std::vector<MovementId>& movements() { return movements_; }
    std::vector<ClusterId>& clusters() { return clusters_; }
    Params& params() { return params_; }

   private:
    NodeId at_;
    Params params_;
    std::vector<MovementId> movements_;  // encodes order of movements
    std::vector<ClusterId> clusters_;
};