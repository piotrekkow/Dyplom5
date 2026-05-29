#pragma once

#include "data/flow.hpp"
#include "data/saturation_override.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"

class DemandData {
   public:
    [[nodiscard]] const SlotArray<MovementTag, Flow>& flows() const {
        return flows_;
    }
    [[nodiscard]] const SlotArray<MovementTag, SaturationOverride>&
    saturationOverrides() const {
        return saturationOverrides_;
    }

    SlotArray<MovementTag, Flow>& flows() { return flows_; }
    SlotArray<MovementTag, SaturationOverride>& saturationOverrides() {
        return saturationOverrides_;
    }

   private:
    SlotArray<MovementTag, Flow> flows_;
    SlotArray<MovementTag, SaturationOverride> saturationOverrides_;
};