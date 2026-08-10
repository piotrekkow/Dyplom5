#pragma once

#include <variant>

#include "crosswalk_signal_group.hpp"
#include "movement_signal_group.hpp"

using SignalGroup = std::variant<CrosswalkSignalGroup, MovementSignalGroup>;