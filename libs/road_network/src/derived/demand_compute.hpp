#pragma once

#include "road_network/demand/derived/flow_ratio.hpp"
#include "road_network/demand/derived/saturation.hpp"
#include "road_network/util/ids.hpp"

class GraphData;
class DemandData;
class Demand;
struct Regulations;

namespace demand_compute {
Saturation baseSaturation(const GraphData& graph, const DemandData& demand,
                          const Regulations& regulations, MovementId id);
std::vector<Saturation> entryLaneSaturations(const GraphData& graph,
                                             const DemandData& demand,
                                             const Regulations& regulations,
                                             EntryId id);
FlowRatio flowRatio(const GraphData& graph, const Demand& demand,
                    MovementId id);

}  // namespace demand_compute