#pragma once

#include <optional>
#include <variant>

#include "data/intersection.hpp"
#include "data/signal_config.hpp"
#include "data/signal_group.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"
#include "road_network/util/slot_map.hpp"

class SignalData {
   public:
    [[nodiscard]] const SlotMap<IntersectionTag, Intersection>& intersections()
        const {
        return intersections_;
    }
    [[nodiscard]] const SlotMap<SignalConfigTag, SignalConfig>& configs()
        const {
        return configs_;
    }
    [[nodiscard]] const SlotMap<SignalGroupTag, SignalGroup>& signalGroups()
        const {
        return signalGroups_;
    }
    SlotMap<IntersectionTag, Intersection>& intersections() {
        return intersections_;
    }
    SlotMap<SignalConfigTag, SignalConfig>& configs() { return configs_; }
    SlotMap<SignalGroupTag, SignalGroup>& signalGroups() {
        return signalGroups_;
    }

    // node → owning intersection (a node is in at most one intersection)
    [[nodiscard]] std::optional<IntersectionId> intersectionAt(
        NodeId id) const {
        const auto* at = nodeIntersection_.get(id);
        return at ? std::optional(*at) : std::nullopt;
    }
    SlotArray<NodeTag, IntersectionId>& nodeIntersection() {
        return nodeIntersection_;
    }

    [[nodiscard]] IntersectionId intersectionOf(SignalGroupId id) const {
        SignalConfigId config = std::visit(
            [](const auto& x) { return x.config(); }, *signalGroups_.get(id));
        return configs_.get(config)->at();
    }

   private:
    SlotMap<IntersectionTag, Intersection> intersections_;
    SlotArray<NodeTag, IntersectionId> nodeIntersection_;
    SlotMap<SignalConfigTag, SignalConfig> configs_;
    SlotMap<SignalGroupTag, SignalGroup> signalGroups_;
};
