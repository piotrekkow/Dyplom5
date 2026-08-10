#include "scene_sync.hpp"

#include <QPainterPath>
#include <QtMath>
#include <cmath>
#include <road_network/graph.hpp>
#include <road_network/graph/derived/graph_geometry.hpp>

#include "bus/network_event_bus.hpp"
#include "items/scene_item_registry.hpp"
#include "root/document.hpp"
#include "ui/core/ui_core.hpp"

SceneSync::SceneSync(UiCore& core, SceneItemRegistry& items, QObject* parent)
    : QObject(parent), core_(core), items_(items) {
    NetworkEventBus& bus = core_.bus;

    subs_.add(bus.nodeCreated, [this](const Created<NodeId>& e) {
        items_.nodes().insert(e.id);
        syncNode(e.id);
    });
    subs_.add(bus.nodeUpdated,
              [this](const Updated<NodeId>& e) { syncNodeDeep(e.id); });
    subs_.add(bus.nodeRemoved, [this](const Removed<NodeId>& e) {
        items_.nodes().remove(e.id);
    });

    subs_.add(bus.entryCreated, [this](const Created<EntryId, NodeId>& e) {
        items_.entries().insert(e.id);
        syncEntry(e.id);
    });
    subs_.add(bus.entryUpdated,
              [this](const Updated<EntryId, NodeId>& e) { syncEntry(e.id); });
    subs_.add(bus.entryRemoved, [this](const Removed<EntryId, NodeId>& e) {
        items_.entries().remove(e.id);
    });

    subs_.add(bus.exitCreated, [this](const Created<ExitId, NodeId>& e) {
        items_.exits().insert(e.id);
        syncExit(e.id);
    });
    subs_.add(bus.exitUpdated,
              [this](const Updated<ExitId, NodeId>& e) { syncExit(e.id); });
    subs_.add(bus.exitRemoved, [this](const Removed<ExitId, NodeId>& e) {
        items_.exits().remove(e.id);
    });

    subs_.add(bus.movementCreated,
              [this](const Created<MovementId, NodeId>& e) {
                  items_.movements().insert(e.id);
                  syncMovement(e.id);
              });
    subs_.add(
        bus.movementUpdated,
        [this](const Updated<MovementId, NodeId>& e) { syncMovement(e.id); });
    subs_.add(bus.movementRemoved,
              [this](const Removed<MovementId, NodeId>& e) {
                  items_.movements().remove(e.id);
              });

    subs_.add(bus.crosswalkSeriesCreated,
              [this](const Created<CrosswalkSeriesId, NodeId>& e) {
                  items_.crosswalkSeries().insert(e.id);
                  syncCrosswalkSeries(e.id);
              });
    subs_.add(bus.crosswalkSeriesUpdated,
              [this](const Updated<CrosswalkSeriesId, NodeId>& e) {
                  syncCrosswalkSeries(e.id);
              });
    subs_.add(bus.crosswalkSeriesRemoved,
              [this](const Removed<CrosswalkSeriesId, NodeId>& e) {
                  items_.crosswalkSeries().remove(e.id);
              });

    subs_.add(bus.crosswalkCreated,
              [this](const Created<CrosswalkId, NodeId>& e) {
                  items_.crosswalks().insert(e.id);
                  syncCrosswalk(e.id);
              });
    subs_.add(
        bus.crosswalkUpdated,
        [this](const Updated<CrosswalkId, NodeId>& e) { syncCrosswalk(e.id); });
    subs_.add(bus.crosswalkRemoved,
              [this](const Removed<CrosswalkId, NodeId>& e) {
                  items_.crosswalks().remove(e.id);
              });

    populate();
}

void SceneSync::populate() {
    const GraphData& g = core_.doc.graph().data();
    for (auto e : g.nodes()) {
        items_.nodes().insert(e.id);
        syncNode(e.id);
    }
    for (auto e : g.entries()) {
        items_.entries().insert(e.id);
        syncEntry(e.id);
    }
    for (auto e : g.exits()) {
        items_.exits().insert(e.id);
        syncExit(e.id);
    }
    for (auto e : g.movements()) {
        items_.movements().insert(e.id);
        syncMovement(e.id);
    }
    for (auto e : g.crosswalkSeries()) {
        items_.crosswalkSeries().insert(e.id);
        syncCrosswalkSeries(e.id);
    }
    for (auto e : g.crosswalks()) {
        items_.crosswalks().insert(e.id);
        syncCrosswalk(e.id);
    }
}

void SceneSync::syncNode(NodeId id) {
    NodeItem* item = items_.nodes().find(id);
    const Node* n = core_.doc.graph().data().nodes().get(id);
    if (!item || !n) return;
    item->setCenter(core_.xf.toScene(n->center()));
}

void SceneSync::syncEntry(EntryId id) {
    EntryItem* item = items_.entries().find(id);
    const Entry* en = core_.doc.graph().data().entries().get(id);
    if (!item || !en) return;

    const QPointF h = core_.xf.toScene(en->params().heading);
    const qreal angleDeg = qRadiansToDegrees(std::atan2(h.y(), h.x()));
    qreal width = 0.0;
    for (const auto& lane : en->params().lanes) width += lane.width;
    item->setGeometry(core_.xf.toScene(en->params().position), angleDeg, width);
}

void SceneSync::syncExit(ExitId id) {
    ExitItem* item = items_.exits().find(id);
    const Exit* ex = core_.doc.graph().data().exits().get(id);
    if (!item || !ex) return;

    const QPointF h = core_.xf.toScene(ex->params().heading);
    const qreal angleDeg = qRadiansToDegrees(std::atan2(h.y(), h.x()));
    qreal width = 0.0;
    for (const double laneWidth : ex->params().lanes) width += laneWidth;
    item->setGeometry(core_.xf.toScene(ex->params().position), angleDeg, width);
}

void SceneSync::syncMovement(MovementId id) {
    MovementItem* item = items_.movements().find(id);
    if (!item || !core_.doc.graph().data().movements().alive(id)) return;
    item->setPaths(core_.xf.toScene(
        graph_geometry::movementPolylines(core_.doc.graph().data(), id)));
}

void SceneSync::syncCrosswalkSeries(CrosswalkSeriesId id) {
    CrosswalkSeriesItem* item = items_.crosswalkSeries().find(id);
    const CrosswalkSeries* cs =
        core_.doc.graph().data().crosswalkSeries().get(id);
    if (!item || !cs) return;

    const Polyline& path = cs->params().criticalGreenPath;
    item->setPath(path.empty() ? QPainterPath{} : core_.xf.toScene(path));
}

void SceneSync::syncCrosswalk(CrosswalkId id) {
    CrosswalkItem* item = items_.crosswalks().find(id);
    const Crosswalk* cw = core_.doc.graph().data().crosswalks().get(id);
    if (!item || !cw) return;

    const CrosswalkGeometry geo = cw->geometry();
    item->setGeometry(core_.xf.toScene(geo.anchor), core_.xf.toScene(geo.split),
                      core_.xf.toScene(geo.edge));
}

void SceneSync::syncNodeDeep(NodeId id) {
    syncNode(id);
    const Node* n = core_.doc.graph().data().nodes().get(id);
    if (!n) return;
    for (EntryId eid : n->entries()) {
        syncEntry(eid);
        const Entry* en = core_.doc.graph().data().entries().get(eid);
        if (!en) continue;
        for (MovementId mid : en->movements()) syncMovement(mid);
    }
    for (ExitId xid : n->exits()) syncExit(xid);
    for (CrosswalkSeriesId csid : n->crosswalkSeries()) {
        syncCrosswalkSeries(csid);
        const CrosswalkSeries* cs =
            core_.doc.graph().data().crosswalkSeries().get(csid);
        if (!cs) continue;
        for (CrosswalkId cid : cs->crosswalks()) syncCrosswalk(cid);
    }
}
