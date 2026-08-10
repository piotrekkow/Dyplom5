#pragma once

#include <common/slot_array.hpp>

#include "derived/flow_ratio.hpp"
#include "derived/saturation.hpp"
#include "road_network/demand/data/saturation_override.hpp"
#include "road_network/util/ids.hpp"

class DemandDerived {
   public:
    [[nodiscard]] const SlotArray<EntryTag, std::vector<Saturation>>&
    laneSaturations() const {
        return laneSaturations_;
    }

    [[nodiscard]] const SlotArray<MovementTag, FlowRatio>& flowRatios() const {
        return flowRatios_;
    }

    // Per-movement base saturation from the regulations (turn class +
    // opposing flow), before any override — what an override replaces.
    [[nodiscard]] const SlotArray<MovementTag, Saturation>& baseSaturations()
        const {
        return baseSaturations_;
    }

    SlotArray<EntryTag, std::vector<Saturation>>& laneSaturations() {
        return laneSaturations_;
    }

    SlotArray<MovementTag, FlowRatio>& flowRatios() { return flowRatios_; }

    SlotArray<MovementTag, Saturation>& baseSaturations() {
        return baseSaturations_;
    }

   private:
    SlotArray<EntryTag, std::vector<Saturation>> laneSaturations_;
    SlotArray<MovementTag, FlowRatio> flowRatios_;
    SlotArray<MovementTag, Saturation> baseSaturations_;
};


