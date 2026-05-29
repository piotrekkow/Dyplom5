#pragma once

#include <QGraphicsView>

class NetworkView : public QGraphicsView {
    Q_OBJECT
   public:
    NetworkView(QWidget* parent = nullptr);

    // Panning
    void panButton(Qt::MouseButton button) { panButton_ = button; }
    [[nodiscard]] bool isPanning() const { return isPanning_; }

    // Zooming
    [[nodiscard]] qreal zoomLevel() const { return zoomLevel_; }
    void zoomLimits(qreal min, qreal max) {
        minZoom_ = min;
        maxZoom_ = max;
    }
    void zoomStrength(qreal s) { zoomStrength_ = s; }
    void resetZoom();

   protected:
    void showEvent(QShowEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

   private:
    // Panning
    bool isPanning_{false};
    QPoint lastPanPos_;
    Qt::MouseButton panButton_{Qt::MiddleButton};
    Qt::CursorShape panCursor_{Qt::ClosedHandCursor};

    void startPanning(const QPoint& p);
    void updatePanning(const QPoint& p);
    void stopPanning();

    // Zooming
    qreal zoomLevel_;
    qreal zoomStrength_;
    qreal minZoom_;
    qreal maxZoom_;
};
