#pragma once

#include "road_network/graph/derived/turn_type.hpp"
#include "road_network/util/geometry/polyline.hpp"
#include "road_network/util/geometry/position.hpp"
#include "road_network/util/ids.hpp"

class GraphData;

namespace graph_geometry {
struct MovementPolyline {
    Polyline poly;
    float stopOffset;
};

std::vector<Position> lanePositions(const GraphData& data, AdapterId at);
std::vector<MovementPolyline> movementPolylines(const GraphData& data,
                                                MovementId id);
float classifyAngle(const GraphData& data, EntryId entry, ExitId exit);
float classifyAngle(const GraphData& data, MovementId id);
TurnType classifyTurn(const GraphData& data, MovementId id);
bool isValidMovementOrder(const GraphData& data, EntryId id);

// opposing threshold - angle in rad relative to original entry heading >= to be
// considered opposing (-x, x) is not opposing.
std::optional<EntryId> findOpposingEntry(const GraphData& data, EntryId id,
                                         float opposingThreshold = 2.35619f);

// opposing threshold - angle in rad relative to original entry heading >= to be
// considered opposing (-x, x) is not opposing.
bool isOpposingEntry(const GraphData& data, MovementId id, MovementId candidate,
                     float opposingThreshold = 2.35619f);
}  // namespace graph_geometry