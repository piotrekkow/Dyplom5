#pragma once

#include <geometry/polyline.hpp>
#include <regulations/types/stream_kind.hpp>
#include <vector>

#include "road_network/util/ids.hpp"

class CrosswalkSeries {
   public:
    struct Params {
        Polyline criticalGreenPath;
        CrosswalkType type;
        // by default stream regulations evac speed for given type
        std::optional<double> criticalGreenSpeedOverride;
    };

    CrosswalkSeries(NodeId at) : at_(at) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] const std::vector<CrosswalkId>& crosswalks() const {
        return crosswalks_;
    }
    [[nodiscard]] const Params& params() const { return params_; }

    std::vector<CrosswalkId>& crosswalks() { return crosswalks_; }
    Params& params() { return params_; }

   private:
    NodeId at_;
    std::vector<CrosswalkId> crosswalks_;
    Params params_;
};