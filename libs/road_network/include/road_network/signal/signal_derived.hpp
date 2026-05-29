#pragma once

#include "derived/intergreen_matrix.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"

class SignalDerived {
   public:
    [[nodiscard]] const SlotArray<NodeTag, IntergreenMatrix>& intergreenMatrix()
        const {
        return intergreenMatrix_;
    }
    SlotArray<NodeTag, IntergreenMatrix>& intergreenMatrix() {
        return intergreenMatrix_;
    }

   private:
    SlotArray<NodeTag, IntergreenMatrix> intergreenMatrix_;
};