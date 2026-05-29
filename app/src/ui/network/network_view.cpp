#include "network_view.hpp"

#include <QMouseEvent>
#include <QScrollBar>
#include <QShowEvent>

NetworkView::NetworkView(QWidget* parent)
    : QGraphicsView(parent),
      zoomLevel_{1.0},
      zoomStrength_{1.004},
      minZoom_{0.1},
      maxZoom_{50.0} {
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::NoDrag);
    setTransformationAnchor(QGraphicsView::NoAnchor);
    setResizeAnchor(QGraphicsView::NoAnchor);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    // scale(10.0, 10.0);
}

void NetworkView::showEvent(QShowEvent* event) {
    QGraphicsView::showEvent(event);
    centerOn(0, 0);
}

void NetworkView::resetZoom() {
    if (scene()) {
        QRectF bounds = scene()->itemsBoundingRect();
        if (bounds.isNull()) {
            centerOn(0, 0);
        } else {
            qreal padding = 50.0;
            bounds = bounds.marginsAdded(
                QMarginsF(padding, padding, padding, padding));

            fitInView(bounds, Qt::KeepAspectRatio);
        }
        zoomLevel_ = transform().m11();
    }
}

void NetworkView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == panButton_) {
        startPanning(event->pos());
        event->accept();
    } else {
        QGraphicsView::mousePressEvent(event);
    }
}

void NetworkView::mouseMoveEvent(QMouseEvent* event) {
    if (isPanning_) {
        updatePanning(event->pos());
        lastPanPos_ = event->pos();
        event->accept();
    } else {
        QGraphicsView::mouseMoveEvent(event);
    }
}

void NetworkView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == panButton_) {
        stopPanning();
        event->accept();
    } else {
        QGraphicsView::mouseReleaseEvent(event);
    }
}

void NetworkView::wheelEvent(QWheelEvent* event) {
    if (int delta = event->angleDelta().y()) {
        qreal zoomFactor = std::pow(zoomStrength_, delta);
        qreal futureZoomLevel = zoomLevel_ * zoomFactor;

        if (futureZoomLevel < minZoom_ || futureZoomLevel > maxZoom_) return;
        zoomLevel_ = futureZoomLevel;

        QPointF targetViewportPos = event->position();
        QPointF targetScenePos = mapToScene(event->position().toPoint());

        scale(zoomFactor, zoomFactor);

        QPointF newViewportPos = mapFromScene(targetScenePos);
        QPointF viewportDelta = targetViewportPos - newViewportPos;

        horizontalScrollBar()->setValue(horizontalScrollBar()->value() -
                                        qRound(viewportDelta.x()));
        verticalScrollBar()->setValue(verticalScrollBar()->value() -
                                      qRound(viewportDelta.y()));
    }
}

void NetworkView::startPanning(const QPoint& p) {
    isPanning_ = true;
    lastPanPos_ = p;
    setCursor(panCursor_);
    setInteractive(false);
}

void NetworkView::updatePanning(const QPoint& p) {
    QPoint delta = p - lastPanPos_;
    int newHorizontal = horizontalScrollBar()->value() - delta.x();
    int newVertical = verticalScrollBar()->value() - delta.y();

    horizontalScrollBar()->setValue(newHorizontal);
    verticalScrollBar()->setValue(newVertical);
}

void NetworkView::stopPanning() {
    isPanning_ = false;
    unsetCursor();
    setInteractive(true);
}
