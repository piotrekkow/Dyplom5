#pragma once

#include "road_network/util/geometry/line.hpp"
#include "road_network/util/ids.hpp"
#include "stream_type.hpp"

class Crosswalk {
   public:
    // Crosswalk parameters. l1, l2 allow for uneven edge line lengths while
    // ensuring they're parallel. l1, l2 may be negative. Width may also be
    // negative, this means l2 is on the other side.
    // Split serves the purpose of a delimiter for pedestrian
    // crossing from cycling crossing.
    //
    //                  -| ^
    //                ---| | x1
    //              -----| v
    // line.p1 x---------x
    //         |---------|
    //         |---------|
    //         |---------|
    // line.p2 x---------x
    //          <------->| ^
    //            width  | | x2
    //                   | v

    struct SegmentParams {
        float width = 4.f;
        float x1 = 0.f;
        float x2 = 0.f;
    };

    struct Params {
        Line line;
        SegmentParams edge;
        SegmentParams split;
    };

    struct Lines {
        Line l1;
        Line split;
        Line l2;
    };

    Crosswalk(CrosswalkSeriesId parent, Line line,
              CrosswalkType type = CrosswalkType::PEDESTRIAN)
        : parent_(parent), type_(type) {
        params_.line = line;
        applyTypeDefaults();
        enforceInvariants();
    }

    [[nodiscard]] CrosswalkSeriesId parent() const { return parent_; }
    [[nodiscard]] const Params& params() const { return params_; }
    [[nodiscard]] CrosswalkType type() const { return type_; }
    [[nodiscard]] const Lines lines() const {
        return {.l1 = params_.line,
                .split = spToLine(params_.split),
                .l2 = spToLine(params_.edge)};
    }

    void setType(CrosswalkType type) {
        type_ = type;
        enforceInvariants();
    }

    void setLine(Line line) { params_.line = line; }

    void setEdge(SegmentParams edge) {
        params_.edge = edge;
        enforceInvariants();
    }

    void setSplit(SegmentParams split) {
        params_.split = split;
        enforceInvariants();
    }

   private:
    CrosswalkSeriesId parent_;
    Params params_;
    CrosswalkType type_;

    void enforceInvariants() {
        if (!hasIndependentSplit()) {
            params_.split = params_.edge;
        }
    }

    void applyTypeDefaults() {
        params_.edge = {.width = 4.f};

        if (hasIndependentSplit()) {
            params_.split = {.width = 2.f};
        } else {
            params_.split = params_.edge;
        }
    }

    [[nodiscard]] bool hasIndependentSplit() const {
        return type_ == CrosswalkType::PEDESTRIAN_AND_CYCLIST;
    }

    [[nodiscard]] Line spToLine(const SegmentParams& sp) const {
        Vector2 dir = params_.line.direction().normalized();
        Vector2 widthVector = dir.perpCw() * sp.width;
        auto p1 = params_.line.p1 + widthVector - dir * sp.x1;
        auto p2 = params_.line.p2 + widthVector + dir * sp.x2;
        return {.p1 = p1, .p2 = p2};
    }
};