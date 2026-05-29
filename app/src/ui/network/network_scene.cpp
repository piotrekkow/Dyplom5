#include "network_scene.hpp"

#include <qnamespace.h>

#include <QPainter>

#include "ui/network/items/crosswalk_item.hpp"

// #include "items/conflict_map_item.h"

NetworkScene::NetworkScene(QObject* parent) : QGraphicsScene(parent) {
    int gridSize = 1;
    int w = 100;
    int h = 100;

    int halfW = w / 2;
    int halfH = h / 2;

    // Vertical lines
    for (int x = -halfW; x <= halfW; x += gridSize) {
        qreal lineWidth = x % 10 == 0 ? 0.1 : 0.05;
        addLine(x, -halfH, x, halfH, QPen(QColor("#242424"), lineWidth));
    }

    // Horizontal lines
    for (int y = -halfH; y <= halfH; y += gridSize) {
        qreal lineWidth = y % 10 == 0 ? 0.1 : 0.03;
        addLine(-halfW, y, halfW, y, QPen(QColor("#242424"), lineWidth));
    }

    addLine(-5, -5, 5, 5, QPen(QColor("#242424"), 0.05));
    addLine(5, -5, -5, 5, QPen(QColor("#242424"), 0.05));
}

const NodeItem* NetworkScene::nodeItem(NodeId id) const {
    auto it = nodeItems_.find(id);
    return it != nodeItems_.end() ? it->second : nullptr;
}
const EntryItem* NetworkScene::entryItem(EntryId id) const {
    auto it = entryItems_.find(id);
    return it != entryItems_.end() ? it->second : nullptr;
}

NodeItem* NetworkScene::nodeItem(NodeId id) {
    auto it = nodeItems_.find(id);
    return it != nodeItems_.end() ? it->second : nullptr;
}

EntryItem* NetworkScene::entryItem(EntryId id) {
    auto it = entryItems_.find(id);
    return it != entryItems_.end() ? it->second : nullptr;
}

const MovementItem* NetworkScene::movementItem(MovementId id) const {
    auto it = movementItems_.find(id);
    return it != movementItems_.end() ? it->second : nullptr;
}

MovementItem* NetworkScene::movementItem(MovementId id) {
    auto it = movementItems_.find(id);
    return it != movementItems_.end() ? it->second : nullptr;
}

void NetworkScene::insertNode(NodeId id, QPointF center) {
    auto* item = new NodeItem(id, center);
    addItem(item);
    nodeItems_[id] = item;
}

void NetworkScene::removeNode(NodeId id) {
    auto it = nodeItems_.find(id);
    if (it == nodeItems_.end()) return;
    removeItem(it->second);
    delete it->second;
    nodeItems_.erase(id);
}

void NetworkScene::insertEntry(EntryId id, QPointF pos, QPointF heading) {
    auto* item = new EntryItem(id, pos, heading);
    addItem(item);
    entryItems_[id] = item;
}
void NetworkScene::removeEntry(EntryId id) {
    auto it = entryItems_.find(id);
    if (it == entryItems_.end()) return;
    removeItem(it->second);
    delete it->second;
    entryItems_.erase(id);
}

const ExitItem* NetworkScene::exitItem(ExitId id) const {
    auto it = exitItems_.find(id);
    return it != exitItems_.end() ? it->second : nullptr;
}

ExitItem* NetworkScene::exitItem(ExitId id) {
    auto it = exitItems_.find(id);
    return it != exitItems_.end() ? it->second : nullptr;
}

void NetworkScene::insertExit(ExitId id, QPointF pos, QPointF heading) {
    auto* item = new ExitItem(id, pos, heading);
    addItem(item);
    exitItems_[id] = item;
}

void NetworkScene::removeExit(ExitId id) {
    auto it = exitItems_.find(id);
    if (it == exitItems_.end()) return;
    removeItem(it->second);
    delete it->second;
    exitItems_.erase(id);
}

void NetworkScene::insertMovement(MovementId id, QPointF nodePos,
                                  const std::vector<QPainterPath>& scenePaths) {
    auto* item = new MovementItem(id, nodePos, scenePaths);
    addItem(item);
    movementItems_[id] = item;
}

void NetworkScene::removeMovement(MovementId id) {
    auto it = movementItems_.find(id);
    if (it == movementItems_.end()) return;
    removeItem(it->second);
    delete it->second;
    movementItems_.erase(it);
}

const CrosswalkSeriesItem* NetworkScene::crosswalkSeriesItem(
    CrosswalkSeriesId id) const {
    auto it = crosswalkSeriesItems_.find(id);
    return it != crosswalkSeriesItems_.end() ? it->second : nullptr;
}

CrosswalkSeriesItem* NetworkScene::crosswalkSeriesItem(CrosswalkSeriesId id) {
    auto it = crosswalkSeriesItems_.find(id);
    return it != crosswalkSeriesItems_.end() ? it->second : nullptr;
}

const CrosswalkItem* NetworkScene::crosswalkItem(CrosswalkId id) const {
    auto it = crosswalkItems_.find(id);
    return it != crosswalkItems_.end() ? it->second : nullptr;
}
CrosswalkItem* NetworkScene::crosswalkItem(CrosswalkId id) {
    auto it = crosswalkItems_.find(id);
    return it != crosswalkItems_.end() ? it->second : nullptr;
}

void NetworkScene::insertCrosswalkSeries(CrosswalkSeriesId id,
                                         const QPainterPath& scenePath) {
    auto* item = new CrosswalkSeriesItem(id);
    item->setPedestrianPath(scenePath);
    addItem(item);
    crosswalkSeriesItems_[id] = item;
}

void NetworkScene::removeCrosswalkSeries(CrosswalkSeriesId id) {
    auto it = crosswalkSeriesItems_.find(id);
    if (it == crosswalkSeriesItems_.end()) return;
    removeItem(it->second);
    delete it->second;
    crosswalkSeriesItems_.erase(it);
}

void NetworkScene::insertCrosswalk(CrosswalkId id, QLineF l1, QLineF split,
                                   QLineF l2) {
    auto* item = new CrosswalkItem(id, l1, split, l2);
    addItem(item);
    crosswalkItems_[id] = item;
}
void NetworkScene::removeCrosswalk(CrosswalkId id) {
    auto it = crosswalkItems_.find(id);
    if (it == crosswalkItems_.end()) return;
    removeItem(it->second);
    delete it->second;
    crosswalkItems_.erase(it);
}

const SignalGroupItem* NetworkScene::signalGroupItem(SignalGroupId id) const {
    auto it = signalGroupItems_.find(id);
    return it != signalGroupItems_.end() ? it->second : nullptr;
}

SignalGroupItem* NetworkScene::signalGroupItem(SignalGroupId id) {
    auto it = signalGroupItems_.find(id);
    return it != signalGroupItems_.end() ? it->second : nullptr;
}

void NetworkScene::insertSignalGroup(SignalGroupId id, QPointF pos,
                                     const QString& label) {
    auto* item = new SignalGroupItem(id, pos, label);
    addItem(item);
    signalGroupItems_[id] = item;
}

void NetworkScene::removeSignalGroup(SignalGroupId id) {
    auto it = signalGroupItems_.find(id);
    if (it == signalGroupItems_.end()) return;
    removeItem(it->second);
    delete it->second;
    signalGroupItems_.erase(it);
}

void NetworkScene::setBackdrop(const QString& path, QPointF origin, qreal mpp) {
    clearBackdrop();
    QPixmap px(path);
    backdropItem_ = addPixmap(px);
    backdropItem_->setScale(mpp);
    backdropItem_->setPos(origin);
    backdropItem_->setZValue(-999);
    backdropItem_->setOpacity(0.15);
}

void NetworkScene::clearBackdrop() {
    if (!backdropItem_) return;
    removeItem(backdropItem_);
    delete backdropItem_;
    backdropItem_ = nullptr;
}