#pragma once

#include <geometry/line.hpp>
#include <regulations/types/stream_kind.hpp>

#include "crosswalk_geometry.hpp"
#include "road_network/util/ids.hpp"

class Crosswalk {
   public:
    // Crosswalk geometry. l1, l2 allow for uneven edge line lengths while
    // ensuring they're parallel. l1, l2 may be negative. Width may also be
    // negative, this means l2 is on the other side.
    // Split serves the purpose of a delimiter for pedestrian
    // crossing from cycling crossing.
    //
    //                    -| ^
    //                  ---| | x1
    //                -----| v
    // anchor.p1 x---------x
    //           |---------|
    //           |---------|
    //           |---------|
    // anchor.p2 x---------x
    //            <------->| ^
    //              width  | | x2
    //                     | v

    struct Params {
        CrosswalkGeometryInput input;
    };

    Crosswalk(CrosswalkSeriesId parent, Line anchor, CrosswalkType type)
        : parent_(parent) {
        params_.input.anchor = anchor;
        applyDefaults(type);
    }

    [[nodiscard]] CrosswalkSeriesId parent() const { return parent_; }
    [[nodiscard]] const CrosswalkGeometry geometry() const {
        return {.anchor = params_.input.anchor,
                .split = segToLine(params_.input.split),
                .edge = segToLine(params_.input.edge)};
    }
    [[nodiscard]] const Params& params() const { return params_; }
    Params& params() { return params_; }

   private:
    CrosswalkSeriesId parent_;
    Params params_;

    void applyDefaults(CrosswalkType type) {
        switch (type) {
            default:
            case CrosswalkType::PEDESTRIAN:
                params_.input.split = {.width = 4.0, .x1 = 0.0, .x2 = 0.0};
                params_.input.edge = {.width = 4.0, .x1 = 0.0, .x2 = 0.0};
                break;
            case CrosswalkType::PEDESTRIAN_AND_CYCLIST:
                params_.input.split = {.width = 4.0, .x1 = 0.0, .x2 = 0.0};
                params_.input.edge = {.width = 6.0, .x1 = 0.0, .x2 = 0.0};
                break;
            case CrosswalkType::CYCLIST:
                params_.input.split = {.width = 3.0, .x1 = 0.0, .x2 = 0.0};
                params_.input.edge = {.width = 3.0, .x1 = 0.0, .x2 = 0.0};
                break;
        }
    }

    [[nodiscard]] Line segToLine(
        const CrosswalkGeometryInput::Segment& seg) const {
        Vector2 dir = params_.input.anchor.direction().normalized();
        Vector2 widthVector = dir.perpCw() * seg.width;
        auto p1 = params_.input.anchor.p1 + widthVector - dir * seg.x1;
        auto p2 = params_.input.anchor.p2 + widthVector + dir * seg.x2;
        return {.p1 = p1, .p2 = p2};
    }
};