#pragma once

#include <common/slot_array.hpp>

#include "derived/plan_derived.hpp"
#include "road_network/util/ids.hpp"

class ControlDerived {
   public:
    [[nodiscard]] const SlotArray<TimingTag, PlanDerived>& plans() const {
        return plans_;
    }

    SlotArray<TimingTag, PlanDerived>& plans() { return plans_; }

   private:
    SlotArray<TimingTag, PlanDerived> plans_;
};
