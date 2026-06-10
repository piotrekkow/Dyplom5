#pragma once

#include "data/timing.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"
#include "road_network/util/slot_map.hpp"

struct ConfigControl {
    std::vector<TimingId> timings;
};

class ControlData {
   public:
    [[nodiscard]] const SlotArray<SignalConfigTag, ConfigControl>&
    configControls() const {
        return configControls_;
    }
    [[nodiscard]] const SlotMap<TimingTag, Timing>& timings() const {
        return timings_;
    }
    SlotArray<SignalConfigTag, ConfigControl>& configControls() {
        return configControls_;
    }
    SlotMap<TimingTag, Timing>& timings() { return timings_; }

   private:
    SlotArray<SignalConfigTag, ConfigControl> configControls_;
    SlotMap<TimingTag, Timing> timings_;
};
