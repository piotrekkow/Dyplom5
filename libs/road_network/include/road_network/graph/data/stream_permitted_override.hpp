#pragma once

#include <unordered_map>

#include "road_network/util/ids.hpp"

using StreamPermittedOverrideMap =
    std::unordered_map<StreamId, std::unordered_map<StreamId, bool>>;
