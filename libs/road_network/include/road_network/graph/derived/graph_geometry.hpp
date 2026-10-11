#pragma once

#include <geometry/polyline.hpp>
#include <optional>

#include "road_network/graph/data/boundary/entry_lane_spec.hpp"
#include "road_network/graph/data/connector/movement_path_spec.hpp"
#include "road_network/util/ids.hpp"

class GraphData;

namespace graph_geometry {
std::vector<Position> lanePositions(const GraphData& data, EntryId id,
                                    bool withStopOffset = true);
std::vector<Position> lanePositions(const GraphData& data, ExitId id);
std::vector<Polyline> entryStopBars(const GraphData& data, EntryId id,
                                    const std::vector<EntryLaneSpec>& lanes);
std::vector<Polyline> exitLaneBars(const GraphData& data, ExitId id,
                                   const std::vector<double>& widths);

// mid - midpoint, dir - heading
struct MovementAnchors {
    Position eMid;
    Vector2 eDir;
    Position xMid;
    Vector2 xDir;
};
std::optional<MovementAnchors> movementAnchors(const GraphData& data,
                                               MovementId id);

std::vector<Polyline> movementPolylines(const GraphData& data, MovementId id);

// get computed instead of the stored one for ui drag previews
std::vector<Polyline> movementPolylines(const GraphData& data, MovementId id,
                                        const MovementPathSpec& spec);
}  // namespace graph_geometry