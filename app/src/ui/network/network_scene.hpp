#pragma once

#include <qgraphicsitem.h>

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <unordered_map>

#include "items/crosswalk_item.hpp"
#include "items/node_item.hpp"
// #include "items/conflict_map_item.h"
#include <QGraphicsPixmapItem>

#include "items/crosswalk_series_item.hpp"
#include "items/entry_item.hpp"
#include "items/exit_item.hpp"
#include "items/movement_item.hpp"
#include "items/signal_group_item.hpp"
#include "road_network/util/geometry/position.hpp"
#include "ui/coord_transform.hpp"

class NetworkScene : public QGraphicsScene {
    Q_OBJECT
   public:
    NetworkScene(QObject* parent = nullptr);
    [[nodiscard]] const NodeItem* nodeItem(NodeId id) const;
    [[nodiscard]] const EntryItem* entryItem(EntryId id) const;
    [[nodiscard]] const ExitItem* exitItem(ExitId id) const;
    [[nodiscard]] const MovementItem* movementItem(MovementId id) const;
    [[nodiscard]] const CrosswalkSeriesItem* crosswalkSeriesItem(
        CrosswalkSeriesId id) const;
    [[nodiscard]] const CrosswalkItem* crosswalkItem(CrosswalkId id) const;
    [[nodiscard]] const SignalGroupItem* signalGroupItem(
        SignalGroupId id) const;

    NodeItem* nodeItem(NodeId id);
    EntryItem* entryItem(EntryId id);
    ExitItem* exitItem(ExitId id);
    MovementItem* movementItem(MovementId id);
    CrosswalkSeriesItem* crosswalkSeriesItem(CrosswalkSeriesId id);
    CrosswalkItem* crosswalkItem(CrosswalkId id);
    SignalGroupItem* signalGroupItem(SignalGroupId id);

    void insertNode(NodeId, QPointF);
    void removeNode(NodeId);

    void insertEntry(EntryId id, QPointF pos, QPointF heading);
    void removeEntry(EntryId id);

    void insertExit(ExitId id, QPointF pos, QPointF heading);
    void removeExit(ExitId id);

    void insertMovement(MovementId id, QPointF nodeCenter,
                        const std::vector<QPainterPath>& scenePaths);
    void removeMovement(MovementId id);

    void insertCrosswalkSeries(CrosswalkSeriesId id,
                               const QPainterPath& scenePath);
    void removeCrosswalkSeries(CrosswalkSeriesId id);

    void insertCrosswalk(CrosswalkId id, QLineF l1, QLineF split, QLineF l2);
    void removeCrosswalk(CrosswalkId id);

    void insertSignalGroup(SignalGroupId id, QPointF pos, const QString& label);
    void removeSignalGroup(SignalGroupId id);

    void setBackdrop(const QString& path, QPointF origin, qreal mpp);
    void clearBackdrop();

   // // debug
   //protected:
   // void mousePressEvent(QGraphicsSceneMouseEvent* event) override {
   //     CoordTransform xf;
   //     QPointF sp = event->scenePos();
   //     Position p = xf.toDomain(sp);
   //     qDebug().noquote().nospace()
   //         << "{.x = " << QString::number(static_cast<double>(p.x), 'f', 1)
   //         << "f, .y = " << QString::number(static_cast<double>(p.y), 'f', 1)
   //         << "f},";

   //     QGraphicsScene::mousePressEvent(event);
   // }
   // // debug

   private:
    std::unordered_map<NodeId, NodeItem*> nodeItems_;
    std::unordered_map<EntryId, EntryItem*> entryItems_;
    std::unordered_map<ExitId, ExitItem*> exitItems_;
    std::unordered_map<MovementId, MovementItem*> movementItems_;
    std::unordered_map<CrosswalkSeriesId, CrosswalkSeriesItem*>
        crosswalkSeriesItems_;
    std::unordered_map<CrosswalkId, CrosswalkItem*> crosswalkItems_;
    std::unordered_map<SignalGroupId, SignalGroupItem*> signalGroupItems_;
    QGraphicsPixmapItem* backdropItem_ = nullptr;
};