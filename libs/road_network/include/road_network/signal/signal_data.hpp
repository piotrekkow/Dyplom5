#pragma once

#include <variant>

#include "data/signal_config.hpp"
#include "data/signal_group.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"
#include "road_network/util/slot_map.hpp"

struct NodeSignal {
    std::vector<SignalConfigId> configs;
};

class SignalData {
   public:
    [[nodiscard]] const SlotArray<NodeTag, NodeSignal>& nodeSignals() const {
        return nodeSignals_;
    }
    [[nodiscard]] const SlotMap<SignalConfigTag, SignalConfig>& configs()
        const {
        return configs_;
    }
    [[nodiscard]] const SlotMap<SignalGroupTag, SignalGroup>& signalGroups()
        const {
        return signalGroups_;
    }
    SlotArray<NodeTag, NodeSignal>& nodeSignals() { return nodeSignals_; }
    SlotMap<SignalConfigTag, SignalConfig>& configs() { return configs_; }
    SlotMap<SignalGroupTag, SignalGroup>& signalGroups() {
        return signalGroups_;
    }

    [[nodiscard]] NodeId nodeOf(SignalGroupId id) const {
        SignalConfigId config = std::visit(
            [](const auto& x) { return x.config(); }, *signalGroups_.get(id));
        return configs_.get(config)->at();
    }

   private:
    SlotArray<NodeTag, NodeSignal> nodeSignals_;
    SlotMap<SignalConfigTag, SignalConfig> configs_;
    SlotMap<SignalGroupTag, SignalGroup> signalGroups_;
};
