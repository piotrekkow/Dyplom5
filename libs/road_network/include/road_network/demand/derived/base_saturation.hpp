#pragma once

#include "road_network/demand/derived/saturation.hpp"
#include "road_network/graph/derived/turn_type.hpp"
#include "road_network/util/ids.hpp"

class GraphData;
class DemandData;

namespace demand_compute {

// Default prot/perm saturation flow for a movement by turn type. Qn is the
// opposing equivalent flow (veh/h) — see opposingFlowQn.
Saturation movementBaseSatTable(TurnType turn, float Qn);

// Opposing equivalent flow Qn for an entry: through + (lane-dependent share
// of) right-turn flow of the opposing entry. 0 if no opposing entry exists.
float opposingFlowQn(const GraphData& graph, const DemandData& demand,
                     EntryId id);

}  // namespace demand_compute
