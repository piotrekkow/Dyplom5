#pragma once

#include <common/slot_array.hpp>
#include <common/slot_map.hpp>

#include "data/boundary/entry.hpp"
#include "data/boundary/exit.hpp"
#include "data/connector/cluster.hpp"
#include "data/connector/movement.hpp"
#include "data/crosswalk/crosswalk.hpp"
#include "data/crosswalk/crosswalk_series.hpp"
#include "data/node.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/stream_matrix.hpp"

class GraphData {
   public:
    [[nodiscard]] const SlotMap<NodeTag, Node>& nodes() const { return nodes_; }
    [[nodiscard]] const SlotMap<EntryTag, Entry>& entries() const {
        return entries_;
    }
    [[nodiscard]] const SlotMap<ExitTag, Exit>& exits() const { return exits_; }
    [[nodiscard]] const SlotMap<MovementTag, Movement>& movements() const {
        return movements_;
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
    SlotMap<EntryTag, Entry>& entries() { return entries_; }
    SlotMap<ExitTag, Exit>& exits() { return exits_; }
    SlotMap<MovementTag, Movement>& movements() { return movements_; }
    SlotMap<CrosswalkSeriesTag, CrosswalkSeries>& crosswalkSeries() {
        return crosswalkSeries_;
    }
    SlotMap<CrosswalkTag, Crosswalk>& crosswalks() { return crosswalks_; }
    SlotMap<ClusterTag, Cluster>& clusters() { return clusters_; }

    [[nodiscard]] const SlotArray<NodeTag, StreamMatrix<bool>>&
    streamPermittedOverrides() const {
        return streamPermittedOverrides_;
    }
    [[nodiscard]] const SlotArray<NodeTag, StreamMatrix<int>>&
    streamIntergreenOverrides() const {
        return streamIntergreenOverrides_;
    }
    SlotArray<NodeTag, StreamMatrix<bool>>& streamPermittedOverrides() {
        return streamPermittedOverrides_;
    }
    SlotArray<NodeTag, StreamMatrix<int>>& streamIntergreenOverrides() {
        return streamIntergreenOverrides_;
    }

   private:
    SlotMap<NodeTag, Node> nodes_;
    SlotMap<EntryTag, Entry> entries_;
    SlotMap<ExitTag, Exit> exits_;
    SlotMap<MovementTag, Movement> movements_;
    SlotMap<CrosswalkSeriesTag, CrosswalkSeries> crosswalkSeries_;
    SlotMap<CrosswalkTag, Crosswalk> crosswalks_;
    SlotMap<ClusterTag, Cluster> clusters_;

    SlotArray<NodeTag, StreamMatrix<bool>> streamPermittedOverrides_;
    SlotArray<NodeTag, StreamMatrix<int>> streamIntergreenOverrides_;
};