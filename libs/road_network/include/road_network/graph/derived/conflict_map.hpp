#pragma once

#include <unordered_map>

#include "conflict_point.hpp"
#include "road_network/util/ids.hpp"

using ConflictMap = std::unordered_map<
    StreamId, std::unordered_map<StreamId, std::vector<ConflictPoint>>>;