#pragma once

#include <optional>
#include <regulations/types/stream_kind.hpp>

#include "entry_section.hpp"
#include "movement_path_spec.hpp"
#include "road_network/util/ids.hpp"

// one movement per entry-exit pair
class Movement {
   public:
    struct Params {
        MovementPathSpec pathSpec{
            .dE = 0.0, .dX = 0.0, .params = QuadBezierParams{}};
        MovementType type = MovementType::VEHICLE;
        std::vector<int> splitPerLane;  // into how many exit lanes does a
                                        // given entry lane split
        [[nodiscard]] StreamKind kind() const { return toStreamKind(type); }
    };

    Movement(EntryId from, ExitId to, EntrySection lanes)
        : from_(from), to_(to), lanes_(lanes) {
        params_.splitPerLane.assign(static_cast<size_t>(lanes.laneCount), 1);
    }

    [[nodiscard]] EntryId from() const { return from_; }
    [[nodiscard]] ExitId to() const { return to_; }
    [[nodiscard]] const EntrySection& lanes() const { return lanes_; }
    [[nodiscard]] std::optional<ClusterId> cluster() const { return cluster_; }
    [[nodiscard]] const Params& params() const { return params_; }

    Params& params() { return params_; }
    void setCluster(std::optional<ClusterId> cluster) { cluster_ = cluster; }

   private:
    EntryId from_;
    ExitId to_;
    EntrySection lanes_;
    std::optional<ClusterId> cluster_;
    Params params_;
};