#pragma once

#include <common/slot_array.hpp>
#include <common/slot_map.hpp>

#include "data/signal_group.hpp"
#include "road_network/util/ids.hpp"

class SignalData {
   public:
    [[nodiscard]] const SlotArray<NodeTag, std::vector<SignalGroupId>>&
    nodeGroups() const {
        return nodeGroups_;
    }
    [[nodiscard]] const SlotMap<SignalGroupTag, SignalGroup>& signalGroups()
        const {
        return signalGroups_;
    }

    SlotArray<NodeTag, std::vector<SignalGroupId>>& nodeGroups() {
        return nodeGroups_;
    }
    SlotMap<SignalGroupTag, SignalGroup>& signalGroups() {
        return signalGroups_;
    }

   private:
    SlotArray<NodeTag, std::vector<SignalGroupId>> nodeGroups_;
    SlotMap<SignalGroupTag, SignalGroup> signalGroups_;
};
