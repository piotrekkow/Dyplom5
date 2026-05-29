#pragma once

#include "derived/conflict_map.hpp"
#include "derived/stream_intergreen_map.hpp"
#include "derived/stream_permitted_map.hpp"
#include "derived/turn_type.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"

// Derived graph data container. Stores derived data. Only mutated via Graph /
// GraphEditor otherwise const access
class GraphDerived {
   public:
    [[nodiscard]] const SlotArray<NodeTag, ConflictMap>& conflict() const {
        return conflict_;
    }
    [[nodiscard]] const SlotArray<NodeTag, StreamIntergreenMap>&
    streamIntergreen() const {
        return streamIntergreen_;
    }
    [[nodiscard]] const SlotArray<NodeTag, StreamPermittedMap>&
    streamPermitted() const {
        return streamPermitted_;
    }
    [[nodiscard]] const SlotArray<MovementTag, TurnType>& turnTypes() const {
        return turnTypes_;
    }

    SlotArray<NodeTag, ConflictMap>& conflict() { return conflict_; }
    SlotArray<NodeTag, StreamIntergreenMap>& streamIntergreen() {
        return streamIntergreen_;
    }
    SlotArray<NodeTag, StreamPermittedMap>& streamPermitted() {
        return streamPermitted_;
    }
    SlotArray<MovementTag, TurnType>& turnTypes() { return turnTypes_; }

   private:
    SlotArray<NodeTag, ConflictMap> conflict_;
    SlotArray<NodeTag, StreamIntergreenMap> streamIntergreen_;
    SlotArray<NodeTag, StreamPermittedMap> streamPermitted_;
    SlotArray<MovementTag, TurnType> turnTypes_;
};