#pragma once

#include "entry_section.hpp"
#include "road_network/util/ids.hpp"

// lane group (obliczeniowa grupa pasów)
class Cluster {
   public:
    Cluster(EntryId at, std::vector<MovementId> movements, EntrySection lanes)
        : at_(at), movements_(std::move(movements)), lanes_(lanes) {}

    [[nodiscard]] EntryId at() const { return at_; }
    [[nodiscard]] const std::vector<MovementId>& movements() const {
        return movements_;
    }
    [[nodiscard]] const EntrySection& lanes() const { return lanes_; }

   private:
    EntryId at_;
    std::vector<MovementId> movements_;
    EntrySection lanes_;
};