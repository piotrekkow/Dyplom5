#pragma once

#include <common/slot_array.hpp>

#include "derived/approach_matrix.hpp"
#include "derived/intergreen_matrix.hpp"
#include "derived/protection_resolution.hpp"
#include "road_network/util/ids.hpp"

class SignalDerived {
   public:
    [[nodiscard]] const SlotArray<NodeTag, IntergreenMatrix>&
    intergreenMatrices() const {
        return intergreenMatrices_;
    }
    SlotArray<NodeTag, IntergreenMatrix>& intergreenMatrices() {
        return intergreenMatrices_;
    }

    [[nodiscard]] const SlotArray<NodeTag, ApproachMatrix>& approachMatrices()
        const {
        return approachMatrices_;
    }
    SlotArray<NodeTag, ApproachMatrix>& approachMatrices() {
        return approachMatrices_;
    }

    [[nodiscard]] const SlotArray<SignalGroupTag, ProtectionResolution>&
    protectionResolutions() const {
        return protectionResolutions_;
    }
    SlotArray<SignalGroupTag, ProtectionResolution>& protectionResolutions() {
        return protectionResolutions_;
    }

   private:
    SlotArray<NodeTag, IntergreenMatrix> intergreenMatrices_;
    SlotArray<NodeTag, ApproachMatrix> approachMatrices_;
    SlotArray<SignalGroupTag, ProtectionResolution> protectionResolutions_;
};
