#pragma once

#include <control/schema/stage.hpp>

#include "road_network/util/ids.hpp"

class Stage {
   public:
    struct Params {
        control::Stage<SignalGroupId> core;
    };
    Stage(NodeId at, control::Stage<SignalGroupId> core) : at_(at) {
        params_.core = std::move(core);
    }

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] const Params& params() const { return params_; }
    Params& params() { return params_; }

   private:
    NodeId at_;
    Params params_;
};