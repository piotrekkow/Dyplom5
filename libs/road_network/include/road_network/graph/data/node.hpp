#pragma once

#include "road_network/util/geometry/position.hpp"
#include "road_network/util/ids.hpp"

class Node {
   public:
    Node(Position center) : center_(center) {}
    [[nodiscard]] const Position& center() const { return center_; }
    [[nodiscard]] const std::vector<EntryId>& entries() const {
        return entries_;
    }
    [[nodiscard]] const std::vector<ExitId>& exits() const { return exits_; }
    [[nodiscard]] const std::vector<CrosswalkSeriesId>& crosswalkSeries()
        const {
        return crosswalkSeries_;
    }

    void setCenter(Position center) { center_ = center; }
    std::vector<EntryId>& entries() { return entries_; }
    std::vector<ExitId>& exits() { return exits_; }
    std::vector<CrosswalkSeriesId>& crosswalkSeries() {
        return crosswalkSeries_;
    }

   private:
    Position center_;
    std::vector<EntryId> entries_;
    std::vector<ExitId> exits_;
    std::vector<CrosswalkSeriesId> crosswalkSeries_;
};
