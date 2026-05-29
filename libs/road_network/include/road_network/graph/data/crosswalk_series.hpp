#pragma once

#include <optional>
#include <vector>

#include "road_network/util/geometry/polyline.hpp"
#include "road_network/util/ids.hpp"

class CrosswalkSeries {
   public:
    CrosswalkSeries(NodeId at) : at_(at) {}

    [[nodiscard]] const std::vector<CrosswalkId>& crosswalks() const {
        return crosswalks_;
    }
    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] float length() const { return pedestrianPath_.length(); }
    [[nodiscard]] const Polyline& pedestrianPath() const {
        return pedestrianPath_;
    }
    [[nodiscard]] float pedestrianWalkSpeed() const {
        return pedestrianWalkSpeed_;
    }

    std::vector<CrosswalkId>& crosswalks() { return crosswalks_; }
    Polyline& pedestrianPath() { return pedestrianPath_; }

   private:
    NodeId at_;
    std::vector<CrosswalkId> crosswalks_;
    Polyline pedestrianPath_;
    float pedestrianWalkSpeed_ = 1.4f;
};