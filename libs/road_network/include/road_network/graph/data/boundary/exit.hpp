#pragma once

#include <geometry/position.hpp>

#include "road_network/util/ids.hpp"

class Exit {
   public:
    struct Params {
        Position position;
        Vector2 heading;
        double speedLimit = 50 / 3.6;  // m/s
        std::vector<double> lanes = {/*width=*/3.5};
    };
    Exit(NodeId at, Position position, Vector2 heading)
        : at_(at), params_(position, heading) {}

    [[nodiscard]] const NodeId at() const { return at_; }
    [[nodiscard]] const Params& params() const { return params_; }

    Params& params() { return params_; }

   private:
    NodeId at_;
    Params params_;
};