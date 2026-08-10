#pragma once

#include <regulations/types/signal_kind.hpp>

#include "road_network/util/ids.hpp"

class CrosswalkSignalGroup {
   public:
    struct Params {
        // Signal editor protects from wrong type input
        SignalKind kind = SignalKind::PEDESTRIAN;
        int intervalMinFixed = 0;
    };

    // crosswalks must belong to the same series
    CrosswalkSignalGroup(NodeId node, std::vector<CrosswalkId> crosswalks,
                         SignalKind kind, int minIntervalFixed)
        : node_(node), crosswalks_(std::move(crosswalks)) {
        params_.kind = kind;
        params_.intervalMinFixed = minIntervalFixed;
    }

    [[nodiscard]] NodeId node() const { return node_; }
    [[nodiscard]] const std::vector<CrosswalkId>& crosswalks() const {
        return crosswalks_;
    }
    [[nodiscard]] std::vector<StreamId> streams() const {
        return {crosswalks_.begin(), crosswalks_.end()};
    }

    [[nodiscard]] const Params& params() const { return params_; }
    Params& params() { return params_; }
    void setParams(Params p) { params_ = p; }

   private:
    NodeId node_;
    std::vector<CrosswalkId> crosswalks_;
    Params params_;
};