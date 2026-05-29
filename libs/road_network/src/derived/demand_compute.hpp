#pragma once

#include "road_network/demand/derived/saturation.hpp"
#include "road_network/util/ids.hpp"

class GraphData;
class DemandData;

namespace demand_compute {
std::unordered_map<EntryLaneId, Saturation> entryLaneSaturation(
    const GraphData& graph, const DemandData& demand, EntryId id);

}  // namespace demand_compute