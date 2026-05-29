#pragma once

#include "data/signal_group.hpp"
#include "road_network/signal/data/timing.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"
#include "road_network/util/slot_map.hpp"

struct NodeSignal {
    std::vector<SignalGroupId> signalGroups;
    std::vector<TimingId> timings;
};

class SignalData {
   public:
    [[nodiscard]] const SlotArray<NodeTag, NodeSignal>& nodeSignals() const {
        return nodeSignals_;
    }
    [[nodiscard]] const SlotMap<SignalGroupTag, SignalGroup>& signalGroups()
        const {
        return signalGroups_;
    }
    [[nodiscard]] const SlotMap<TimingTag, Timing>& timings() const {
        return timings_;
    }
    SlotArray<NodeTag, NodeSignal>& nodeSignals() { return nodeSignals_; }
    SlotMap<SignalGroupTag, SignalGroup>& signalGroups() {
        return signalGroups_;
    }
    SlotMap<TimingTag, Timing>& timings() { return timings_; }

   private:
    SlotArray<NodeTag, NodeSignal> nodeSignals_;
    SlotMap<SignalGroupTag, SignalGroup> signalGroups_;
    SlotMap<TimingTag, Timing> timings_;
};