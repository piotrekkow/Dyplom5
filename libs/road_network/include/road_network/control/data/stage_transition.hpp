#pragma once

#include <control/schema/stage_transition.hpp>

#include "road_network/util/ids.hpp"

class StageTransition {
   public:
    struct Params {
        control::StageTransition<SignalGroupId> core;
    };
    StageTransition(NodeId at, StageId from, StageId to)
        : at_(at), from_(from), to_(to) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] StageId from() const { return from_; }
    [[nodiscard]] StageId to() const { return to_; }
    [[nodiscard]] const Params& params() const { return params_; }
    Params& params() { return params_; }

   private:
    NodeId at_;
    Params params_;
    StageId from_;
    StageId to_;
};