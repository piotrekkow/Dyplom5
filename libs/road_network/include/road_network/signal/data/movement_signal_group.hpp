#pragma once

#include <regulations/types/signal_kind.hpp>

#include "road_network/util/ids.hpp"

class MovementSignalGroup {
   public:
    struct Params {
        SignalKind kind = SignalKind::VEHICLE;  // vehicle | bus | tram
        bool requestProtected = true;
        int intervalMinFixed = 0;
    };

    MovementSignalGroup(NodeId node, EntryId entry,
                        std::vector<ClusterId> clusters,
                        std::vector<MovementId> movements, int minIntervalFixed)
        : node_(node),
          entry_(entry),
          clusters_(std::move(clusters)),
          movements_(std::move(movements)) {
        params_.intervalMinFixed = minIntervalFixed;
    }

    [[nodiscard]] NodeId node() const { return node_; }
    [[nodiscard]] EntryId entry() const { return entry_; }
    [[nodiscard]] const std::vector<ClusterId>& clusters() const {
        return clusters_;
    }
    [[nodiscard]] std::vector<StreamId> streams() const {
        return {movements_.begin(), movements_.end()};
    }
    [[nodiscard]] const std::vector<MovementId> movements() const {
        return movements_;
    }

    [[nodiscard]] const Params& params() const { return params_; }
    Params& params() { return params_; }
    void setParams(Params p) { params_ = p; }

   private:
    NodeId node_;
    EntryId entry_;
    std::vector<ClusterId> clusters_;
    std::vector<MovementId> movements_;
    Params params_;
};