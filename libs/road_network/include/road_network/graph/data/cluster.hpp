#pragma once

#include <utility>

#include "road_network/util/ids.hpp"

// lane group (obliczeniowa grupa pasów)
class Cluster {
   public:
    Cluster(EntryId at, std::vector<MovementId> movements,
            std::vector<EntryLaneId> lanes)
        : at_(at), movements_(std::move(movements)), lanes_(std::move(lanes)) {}

    [[nodiscard]] EntryId at() const { return at_; }
    [[nodiscard]] const std::vector<MovementId>& movements() const {
        return movements_;
    }
    [[nodiscard]] const std::vector<EntryLaneId>& lanes() const {
        return lanes_;
    }

   private:
    EntryId at_;
    std::vector<MovementId> movements_;
    std::vector<EntryLaneId> lanes_;
};