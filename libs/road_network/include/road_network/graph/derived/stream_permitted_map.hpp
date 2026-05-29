#pragma once

#include <unordered_map>
#include <unordered_set>

#include "road_network/util/ids.hpp"

using StreamPermittedMap =
    std::unordered_map<StreamId, std::unordered_set<StreamId>>;