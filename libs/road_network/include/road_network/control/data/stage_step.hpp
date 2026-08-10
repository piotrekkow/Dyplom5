#pragma once

#include <control/schema/stage_step.hpp>

#include "road_network/util/ids.hpp"

class StageStep {
   public:
    StageStep(NodeId at, StageId stage, StageTransitionId transition,
              int stageDuration)
        : at_(at),
          stage_(stage),
          transition_(transition),
          stageDuration_(stageDuration) {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] StageId stage() const { return stage_; }
    [[nodiscard]] StageTransitionId transition() const { return transition_; }
    [[nodiscard]] int stageDuration() const { return stageDuration_; }

   private:
    NodeId at_;
    StageId stage_;
    StageTransitionId transition_;
    int stageDuration_;
};
