#pragma once

#include <string>

#include "road_network/util/ids.hpp"
#include "stage_step.hpp"

class Timing {
   public:
    struct Params {
        std::string name;
        std::vector<StageStep> steps;
    };

    Timing(NodeId at, std::string name)
        : at_(at), params_{.name = std::move(name)} {}

    [[nodiscard]] NodeId at() const { return at_; }
    [[nodiscard]] const Params& params() const { return params_; }
    Params& params() { return params_; }

   private:
    NodeId at_;
    Params params_;
};