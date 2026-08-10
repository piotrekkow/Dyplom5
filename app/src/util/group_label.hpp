#pragma once

#include <road_network/util/ids.hpp>
#include <string>

class SignalData;

namespace labels {
std::string groupLabel(SignalGroupId id, const SignalData& signal);
}  // namespace labels
