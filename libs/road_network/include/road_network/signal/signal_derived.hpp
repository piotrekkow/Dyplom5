#pragma once

#include "derived/intergreen_matrix.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"

class SignalDerived {
   public:
    [[nodiscard]] const SlotArray<SignalConfigTag, IntergreenMatrix>&
    intergreenMatrix() const {
        return intergreenMatrix_;
    }
    SlotArray<SignalConfigTag, IntergreenMatrix>& intergreenMatrix() {
        return intergreenMatrix_;
    }

   private:
    SlotArray<SignalConfigTag, IntergreenMatrix> intergreenMatrix_;
};
