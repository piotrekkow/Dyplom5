#pragma once

#include "data/crosswalk.hpp"
#include "data/crosswalk_series.hpp"
// #include "data/edge.hpp"
#include "data/cluster.hpp"
#include "data/entry.hpp"
#include "data/entry_lane.hpp"
#include "data/exit.hpp"
#include "data/exit_lane.hpp"
#include "data/movement.hpp"
#include "data/node.hpp"
#include "data/stream_permitted_override.hpp"
#include "derived/stream_intergreen_map.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/slot_array.hpp"
#include "road_network/util/slot_map.hpp"

// Graph data container. Responsibility: own and manage id of each entity.
// Only mutated via Graph / GraphEditor otherwise const access

class GraphData {
   public:
    [[nodiscard]] const SlotMap<NodeTag, Node>& nodes() const { return nodes_; }
    // const SlotMap<EdgeTag, Edge>& edges() const { return edges_; }
    [[nodiscard]] const SlotMap<EntryTag, Entry>& entries() const {
        return entries_;
    }
    [[nodiscard]] const SlotMap<ExitTag, Exit>& exits() const { return exits_; }
    [[nodiscard]] const SlotMap<MovementTag, Movement>& movements() const {
        return movements_;
    }
    [[nodiscard]] const SlotMap<EntryLaneTag, EntryLane>& entryLanes() const {
        return entryLanes_;
    }
    [[nodiscard]] const SlotMap<ExitLaneTag, ExitLane>& exitLanes() const {
        return exitLanes_;
    }
    [[nodiscard]] const SlotMap<CrosswalkSeriesTag, CrosswalkSeries>&
    crosswalkSeries() const {
        return crosswalkSeries_;
    }
    [[nodiscard]] const SlotMap<CrosswalkTag, Crosswalk>& crosswalks() const {
        return crosswalks_;
    }
    [[nodiscard]] const SlotMap<ClusterTag, Cluster>& clusters() const {
        return clusters_;
    }

    SlotMap<NodeTag, Node>& nodes() { return nodes_; }
    // SlotMap<EdgeTag, Edge>& edges() { return edges_; }
    SlotMap<EntryTag, Entry>& entries() { return entries_; }
    SlotMap<ExitTag, Exit>& exits() { return exits_; }
    SlotMap<MovementTag, Movement>& movements() { return movements_; }
    SlotMap<EntryLaneTag, EntryLane>& entryLanes() { return entryLanes_; }
    SlotMap<ExitLaneTag, ExitLane>& exitLanes() { return exitLanes_; }
    SlotMap<CrosswalkSeriesTag, CrosswalkSeries>& crosswalkSeries() {
        return crosswalkSeries_;
    }
    SlotMap<CrosswalkTag, Crosswalk>& crosswalks() { return crosswalks_; }
    SlotMap<ClusterTag, Cluster>& clusters() { return clusters_; }

    [[nodiscard]] const SlotArray<NodeTag, StreamPermittedOverrideMap>&
    streamPermittedOverrides() const {
        return streamPermittedOverrides_;
    }
    SlotArray<NodeTag, StreamPermittedOverrideMap>& streamPermittedOverrides() {
        return streamPermittedOverrides_;
    }

    [[nodiscard]] const SlotArray<NodeTag, StreamIntergreenOverrideMap>&
    streamIntergreenOverrides() const {
        return streamIntergreenOverrides_;
    }
    SlotArray<NodeTag, StreamIntergreenOverrideMap>& streamIntergreenOverrides() {
        return streamIntergreenOverrides_;
    }

   private:
    SlotMap<NodeTag, Node> nodes_;
    // SlotMap<EdgeTag, Edge> edges_;
    SlotMap<EntryTag, Entry> entries_;
    SlotMap<ExitTag, Exit> exits_;
    SlotMap<MovementTag, Movement> movements_;
    SlotMap<EntryLaneTag, EntryLane> entryLanes_;
    SlotMap<ExitLaneTag, ExitLane> exitLanes_;
    SlotMap<CrosswalkSeriesTag, CrosswalkSeries> crosswalkSeries_;
    SlotMap<CrosswalkTag, Crosswalk> crosswalks_;
    SlotMap<ClusterTag, Cluster> clusters_;
    SlotArray<NodeTag, StreamPermittedOverrideMap> streamPermittedOverrides_;
    SlotArray<NodeTag, StreamIntergreenOverrideMap> streamIntergreenOverrides_;
};
