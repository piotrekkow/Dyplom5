#include "network_scene_populator.hpp"

#include <qhashfunctions.h>

#include <road_network/graph/graph_data.hpp>
#include <road_network/graph/graph_derived.hpp>
#include <road_network/signal/signal_data.hpp>

#include "bus/network_events.hpp"
#include "network_scene.hpp"
#include "road_network/graph/derived/conflict_map.hpp"
#include "road_network/graph/derived/graph_geometry.hpp"
#include "road_network/signal/data/signal_group.hpp"
#include "road_network/util/overloaded.hpp"

namespace {

// Returns the scene position for the signal group label.
// For movement groups: mean of served movement entry positions, offset
// backward. For crosswalk groups: mean of crosswalk midpoints. For conditional
// turn arrow groups: entry position offset backward.
QPointF signalGroupLabelPos(const SignalGroup& sg, const GraphData& graphData,
                            CoordTransform xf) {
    static constexpr float kOffsetMeters = 5.0f;

    return std::visit(
        [&](const auto& group) -> QPointF {
            using T = std::decay_t<decltype(group)>;

            if constexpr (std::is_same_v<T, MovementSignalGroup>) {
                auto streams = group.streams();
                if (streams.empty()) return {};
                Position sumPos{.x = 0.f, .y = 0.f};
                Vector2 heading{.dx = 0.f, .dy = 0.f};
                float count = 0.f;
                for (const auto& s : streams) {
                    auto movId = std::get<MovementId>(s);
                    const auto* mov = graphData.movements().get(movId);
                    if (!mov) continue;
                    auto polys =
                        graph_geometry::movementPolylines(graphData, movId);
                    for (const auto& p : polys) {
                        sumPos.x += p.poly.start().x;
                        sumPos.y += p.poly.start().y;
                        count += 1.f;
                    }
                    heading = graphData.entries().get(mov->from())->heading();
                }
                if (count == 0.f) return {};
                Position mean{.x = sumPos.x / count, .y = sumPos.y / count};
                return xf.toScene(mean - (heading * kOffsetMeters));

            } else if constexpr (std::is_same_v<T, CrosswalkSignalGroup>) {
                const auto& cws = group.crosswalks();
                if (cws.empty()) return {};
                Position sumPos{.x = 0.f, .y = 0.f};
                float count = 0.f;
                for (auto cwId : cws) {
                    const auto* cw = graphData.crosswalks().get(cwId);
                    if (!cw) continue;
                    auto line = cw->params().line;
                    sumPos.x += (line.p1.x + line.p2.x) / 2.0f;
                    sumPos.y += (line.p1.y + line.p2.y) / 2.0f;
                    count += 1.f;
                }
                if (count == 0.f) return {};
                return xf.toScene(
                    Position{.x = sumPos.x / count, .y = sumPos.y / count});

            } else {  // ConditionalTurnArrowSignalGroup
                const auto* mov = graphData.movements().get(group.movement());
                if (!mov) return {};
                const auto* entry = graphData.entries().get(mov->from());
                if (!entry) return {};
                return xf.toScene(entry->position() -
                                  entry->heading() * kOffsetMeters);
            }
        },
        sg);
}

QString signalGroupLabelString(const SignalGroup& sg, int index) {
    return std::visit(
        overloaded{
            [&](MovementSignalGroup msg) { return QString("%1K").arg(index); },
            [&](CrosswalkSignalGroup csg) { return QString("%1P").arg(index); },
            [&](ConditionalTurnArrowSignalGroup tsg) {
                return QString("%1S").arg(index);
            }},
        sg);
}

std::vector<Position> conflictPositions(const ConflictMap& map) {
    std::vector<Position> result;
    for (const auto& [_sg, row] : map)
        for (const auto& [_id, points] : row)
            for (const auto& p : points) result.push_back(p.position);
    return result;
}
std::vector<Polyline> movPolylines(const GraphData& data, MovementId id) {
    auto movPolys = graph_geometry::movementPolylines(data, id);
    std::vector<Polyline> polys;
    polys.reserve(movPolys.size());
    for (const auto& mp : movPolys) polys.push_back(mp.poly);
    return polys;
}
}  // namespace

NetworkScenePopulator::NetworkScenePopulator(NetworkScene& scene,
                                             NetworkEventBus& bus,
                                             const GraphData& data,
                                             const GraphDerived& derived,
                                             const SignalData& signalData,
                                             CoordTransform xf)
    : scene_(scene),
      data_(data),
      derived_(derived),
      signalData_(signalData),
      xf_(xf),
      nodeCreated_(bus.nodeCreated.subscribe([this](NodeCreated e) {
          auto node = data_.nodes().get(e.id);
          scene_.insertNode(e.id, xf_.toScene(node->center()));
      })),
      nodeRederived_(bus.nodeRederived.subscribe([this](NodeRederived e) {
          scene_.nodeItem(e.id)->setConflictPoints(
              xf_.toScene(conflictPositions(*derived_.conflict().get(e.id))));
      })),
      nodeRemoved_(bus.nodeRemoved.subscribe(
          [this](NodeRemoved e) { scene_.removeNode(e.id); })),
      entryCreated_(bus.entryCreated.subscribe([this](EntryCreated e) {
          auto entry = data_.entries().get(e.id);
          scene_.insertEntry(e.id, xf_.toScene(entry->position()),
                             xf_.toScene(entry->heading()));
      })),
      entryMoved_(bus.entryMoved.subscribe([this](EntryMoved e) {
          auto entry = data_.entries().get(e.id);
          if (auto* item = scene_.entryItem(e.id)) {
              item->setPlacement(xf_.toScene(entry->position()),
                                 xf_.toScene(entry->heading()));
              item->setLanePoints(
                  xf_.toScene(graph_geometry::lanePositions(data_, e.id)));
          }
      })),
      entryLanesChanged_(
          bus.entryLanesChanged.subscribe([this](EntryLanesChanged e) {
              scene_.entryItem(e.id)->setLanePoints(
                  xf_.toScene(graph_geometry::lanePositions(data_, e.id)));
          })),
      entryRemoved_(bus.entryRemoved.subscribe(
          [this](EntryRemoved e) { scene_.removeEntry(e.id); })),
      exitCreated_(bus.exitCreated.subscribe([this](ExitCreated e) {
          auto exit = data_.exits().get(e.id);
          scene_.insertExit(e.id, xf_.toScene(exit->position()),
                            xf_.toScene(exit->heading()));
      })),
      exitMoved_(bus.exitMoved.subscribe([this](ExitMoved e) {
          auto exit = data_.exits().get(e.id);
          if (auto* item = scene_.exitItem(e.id)) {
              item->setPlacement(xf_.toScene(exit->position()),
                                 xf_.toScene(exit->heading()));
              item->setLanePoints(
                  xf_.toScene(graph_geometry::lanePositions(data_, e.id)));
          }
      })),
      exitLanesChanged_(
          bus.exitLanesChanged.subscribe([this](ExitLanesChanged e) {
              scene_.exitItem(e.id)->setLanePoints(
                  xf_.toScene(graph_geometry::lanePositions(data_, e.id)));
          })),
      exitRemoved_(bus.exitRemoved.subscribe(
          [this](ExitRemoved e) { scene_.removeExit(e.id); })),
      crosswalkSeriesCreated_(bus.crosswalkSeriesCreated.subscribe(
          [this](CrosswalkSeriesCreated e) {
              scene_.insertCrosswalkSeries(
                  e.id,
                  xf_.toScene(
                      data_.crosswalkSeries().get(e.id)->pedestrianPath()));
          })),
      crosswalkSeriesUpdated_(bus.crosswalkSeriesUpdated.subscribe(
          [this](CrosswalkSeriesUpdated e) {
              scene_.crosswalkSeriesItem(e.id)->setPedestrianPath(xf_.toScene(
                  data_.crosswalkSeries().get(e.id)->pedestrianPath()));
          })),
      crosswalkSeriesRemoved_(bus.crosswalkSeriesRemoved.subscribe(
          [this](CrosswalkSeriesRemoved e) {
              scene_.removeCrosswalkSeries(e.id);
          })),
      crosswalkCreated_(
          bus.crosswalkCreated.subscribe([this](CrosswalkCreated e) {
              auto cwLines = data_.crosswalks().get(e.id)->lines();
              scene_.insertCrosswalk(e.id, xf_.toScene(cwLines.l1),
                                     xf_.toScene(cwLines.split),
                                     xf_.toScene(cwLines.l2));
          })),
      crosswalkUpdated_(
          bus.crosswalkUpdated.subscribe([this](CrosswalkUpdated e) {
              auto cwLines = data_.crosswalks().get(e.id)->lines();
              scene_.crosswalkItem(e.id)->setLines(xf_.toScene(cwLines.l1),
                                                   xf_.toScene(cwLines.split),
                                                   xf_.toScene(cwLines.l2));
          })),
      crosswalkRemoved_(bus.crosswalkRemoved.subscribe(
          [this](CrosswalkRemoved e) { scene_.removeCrosswalk(e.id); })),
      movementCreated_(bus.movementCreated.subscribe([this](MovementCreated e) {
          auto node = data_.nodes().get(e.at);
          scene_.insertMovement(e.id, xf_.toScene(node->center()),
                                xf_.toScene(movPolylines(data_, e.id)));
      })),
      movementUpdated_(bus.movementUpdated.subscribe([this](MovementUpdated e) {
          scene_.movementItem(e.id)->setPaths(
              xf_.toScene(movPolylines(data_, e.id)));
      })),
      movementRemoved_(bus.movementRemoved.subscribe(
          [this](MovementRemoved e) { scene_.removeMovement(e.id); })),
      signalGroupCreated_(
          bus.signalGroupCreated.subscribe([this](SignalGroupCreated e) {
              const auto* sg = signalData_.signalGroups().get(e.id);
              if (!sg) return;
              QPointF pos = signalGroupLabelPos(*sg, data_, xf_);

              QString label = signalGroupLabelString(*sg, e.id.index());
              scene_.insertSignalGroup(e.id, pos, label);
          })),
      signalGroupRemoved_(bus.signalGroupRemoved.subscribe(
          [this](SignalGroupRemoved e) { scene_.removeSignalGroup(e.id); })),
      intersectionCreated_(
          bus.intersectionCreated.subscribe([this](IntersectionCreated e) {
              scene_.insertIntersection(e.id, intersectionOutline(e.id));
          })),
      intersectionNodesChanged_(bus.intersectionNodesChanged.subscribe(
          [this](IntersectionNodesChanged e) {
              if (auto* item = scene_.intersectionItem(e.id))
                  item->setOutline(intersectionOutline(e.id));
          })),
      intersectionRemoved_(bus.intersectionRemoved.subscribe(
          [this](IntersectionRemoved e) { scene_.removeIntersection(e.id); })) {
}

// Bounding outline of the intersection's member-node geometry: node centers,
// entry/exit anchors, and crosswalk endpoints, padded.
QPainterPath NetworkScenePopulator::intersectionOutline(
    IntersectionId id) const {
    static constexpr qreal kPaddingMeters = 4.0;
    static constexpr qreal kCornerRadius = 3.0;

    QPainterPath path;
    const auto* intersection = signalData_.intersections().get(id);
    if (!intersection) return path;

    QRectF bounds;
    auto include = [&](QPointF p) {
        QRectF r(p, p);
        bounds = bounds.isNull() ? r : bounds.united(r);
    };

    for (NodeId nodeId : intersection->nodes()) {
        const auto* node = data_.nodes().get(nodeId);
        if (!node) continue;
        include(xf_.toScene(node->center()));
        for (EntryId eId : node->entries())
            include(xf_.toScene(data_.entries().get(eId)->position()));
        for (ExitId xId : node->exits())
            include(xf_.toScene(data_.exits().get(xId)->position()));
        for (CrosswalkSeriesId csId : node->crosswalkSeries()) {
            for (CrosswalkId cwId :
                 data_.crosswalkSeries().get(csId)->crosswalks()) {
                auto line = data_.crosswalks().get(cwId)->params().line;
                include(xf_.toScene(line.p1));
                include(xf_.toScene(line.p2));
            }
        }
    }
    if (bounds.isNull()) return path;

    bounds.adjust(-kPaddingMeters, -kPaddingMeters, kPaddingMeters,
                  kPaddingMeters);
    path.addRoundedRect(bounds, kCornerRadius, kCornerRadius);
    return path;
}