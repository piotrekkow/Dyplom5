#pragma once

#include "road_network/util/ids.hpp"

struct EntrySection {
    EntryId at;
    int firstIdx;
    int laneCount;
    [[nodiscard]] int lastIdx() const { return firstIdx + laneCount - 1; }
};