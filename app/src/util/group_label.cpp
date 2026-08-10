#include "group_label.hpp"

#include <common/overloaded.hpp>
#include <format>
#include <road_network/signal/signal_data.hpp>
#include <variant>

#include "road_network/util/ids.hpp"

namespace labels {
std::string groupLabel(SignalGroupId id, const SignalData& signal) {
    const char* suffix = "?";
    if (const SignalGroup* g = signal.signalGroups().get(id)) {
        const auto kind =
            std::visit([](const auto& sg) { return sg.params().kind; }, *g);
        switch (kind) {
            case SignalKind::VEHICLE:
            case SignalKind::CONDITIONAL_ARROW:
                suffix = "K";
                break;
            case SignalKind::BUS:
                suffix = "B";
                break;
            case SignalKind::TRAM:
                suffix = "T";
                break;
            case SignalKind::PEDESTRIAN:
                suffix = "P";
                break;
            case SignalKind::CYCLIST:
                suffix = "R";
                break;
            case SignalKind::PEDESTRIAN_AND_CYCLIST:
                suffix = "PR";
                break;
            default:
                break;
        }
    }
    return std::format("{}{}", id.index(), suffix);
}
}  // namespace labels
