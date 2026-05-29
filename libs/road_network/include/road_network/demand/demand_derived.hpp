#pragma once

#include "derived/saturation.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"

class DemandDerived {
   public:
    [[nodiscard]] const SlotArray<EntryLaneTag, Saturation>& laneSaturations()
        const {
        return laneSaturations_;
    }

    SlotArray<EntryLaneTag, Saturation>& laneSaturations() {
        return laneSaturations_;
    }

   private:
    SlotArray<EntryLaneTag, Saturation> laneSaturations_;
};