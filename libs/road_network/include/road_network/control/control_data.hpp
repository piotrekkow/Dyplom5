#pragma once

#include <common/slot_array.hpp>
#include <common/slot_map.hpp>

#include "data/node_control.hpp"
#include "data/stage.hpp"
#include "data/stage_transition.hpp"
#include "data/timing.hpp"
#include "road_network/util/ids.hpp"

class ControlData {
   public:
    [[nodiscard]] const SlotArray<NodeTag, NodeControl>& nodeControls() const {
        return nodeControls_;
    }
    [[nodiscard]] const SlotMap<TimingTag, Timing>& timings() const {
        return timings_;
    }
    [[nodiscard]] const SlotMap<StageTag, Stage>& stages() const {
        return stages_;
    }
    [[nodiscard]] const SlotMap<StageTransitionTag, StageTransition>&
    stageTransitions() const {
        return stageTransitions_;
    }
    SlotArray<NodeTag, NodeControl>& nodeControls() { return nodeControls_; }
    SlotMap<TimingTag, Timing>& timings() { return timings_; }
    SlotMap<StageTag, Stage>& stages() { return stages_; }
    SlotMap<StageTransitionTag, StageTransition>& stageTransitions() {
        return stageTransitions_;
    }

   private:
    SlotArray<NodeTag, NodeControl> nodeControls_;
    SlotMap<TimingTag, Timing> timings_;
    SlotMap<StageTag, Stage> stages_;
    SlotMap<StageTransitionTag, StageTransition> stageTransitions_;
};
