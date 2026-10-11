#pragma once

#include <common/slot_array.hpp>

#include "derived/conflict_point.hpp"
#include "derived/intergreen_diagnostic.hpp"
#include "road_network/util/ids.hpp"
#include "road_network/util/stream_matrix.hpp"

class GraphDerived {
   public:
    [[nodiscard]] const SlotArray<NodeTag,
                                  StreamMatrix<std::vector<ConflictPoint>>>&
    conflicts() const {
        return conflicts_;
    }
    [[nodiscard]] const SlotArray<NodeTag, StreamMatrix<int>>&
    streamIntergreens() const {
        return streamIntergreens_;
    }

    [[nodiscard]] const SlotArray<NodeTag, StreamMatrix<IntergreenDiagnostic>>&
    streamIntergreenDiagnostics() const {
        return streamIntergreenDiagnostics_;
    }

    [[nodiscard]] const SlotArray<NodeTag, StreamMatrix<bool>>&
    streamPermitted() const {
        return streamPermitted_;
    }

    [[nodiscard]] const SlotArray<NodeTag, StreamMatrix<int>>& streamApproach()
        const {
        return streamApproach_;
    }

    SlotArray<NodeTag, StreamMatrix<std::vector<ConflictPoint>>>& conflicts() {
        return conflicts_;
    }
    SlotArray<NodeTag, StreamMatrix<int>>& streamIntergreens() {
        return streamIntergreens_;
    }

    SlotArray<NodeTag, StreamMatrix<IntergreenDiagnostic>>&
    streamIntergreenDiagnostics() {
        return streamIntergreenDiagnostics_;
    }

    SlotArray<NodeTag, StreamMatrix<bool>>& streamPermitted() {
        return streamPermitted_;
    }

    SlotArray<NodeTag, StreamMatrix<int>>& streamApproach() {
        return streamApproach_;
    }

   private:
    SlotArray<NodeTag, StreamMatrix<std::vector<ConflictPoint>>> conflicts_;
    SlotArray<NodeTag, StreamMatrix<int>> streamIntergreens_;
    SlotArray<NodeTag, StreamMatrix<IntergreenDiagnostic>>
        streamIntergreenDiagnostics_;
    SlotArray<NodeTag, StreamMatrix<bool>> streamPermitted_;
    SlotArray<NodeTag, StreamMatrix<int>> streamApproach_;
};