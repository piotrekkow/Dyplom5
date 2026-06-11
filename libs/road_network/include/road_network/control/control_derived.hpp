#pragma once

#include <vector>

#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"

// Validity flags of one timing against current signal-domain parameters
// (group intervalMin limits, config intergreen matrix). Violations are
// flagged, never auto-deleted.
struct TimingValidity {
    bool intervalMinOk = true;
    bool intergreenOk = true;
    [[nodiscard]] bool ok() const { return intervalMinOk && intergreenOk; }
};

// Streams of the config's intersection not covered by any of its groups.
struct ConfigCompleteness {
    std::vector<ClusterId> uncoveredClusters;
    std::vector<CrosswalkId> uncoveredCrosswalks;
    [[nodiscard]] bool complete() const {
        return uncoveredClusters.empty() && uncoveredCrosswalks.empty();
    }
};

class ControlDerived {
   public:
    [[nodiscard]] const SlotArray<TimingTag, TimingValidity>& timingValidity()
        const {
        return timingValidity_;
    }
    [[nodiscard]] const SlotArray<SignalConfigTag, ConfigCompleteness>&
    configCompleteness() const {
        return configCompleteness_;
    }

    SlotArray<TimingTag, TimingValidity>& timingValidity() {
        return timingValidity_;
    }
    SlotArray<SignalConfigTag, ConfigCompleteness>& configCompleteness() {
        return configCompleteness_;
    }

   private:
    SlotArray<TimingTag, TimingValidity> timingValidity_;
    SlotArray<SignalConfigTag, ConfigCompleteness> configCompleteness_;
};
