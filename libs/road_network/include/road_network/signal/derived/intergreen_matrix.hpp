#pragma once

#include <unordered_map>

#include "road_network/util/ids.hpp"

using IntergreenMatrix =
    std::unordered_map<SignalGroupId, std::unordered_map<SignalGroupId, int>>;

// class IntergreenMatrix {
//    public:
//     [[nodiscard]] int get(SignalGroupId evacId, SignalGroupId apprId) const;
//     void set(SignalGroupId evacId, SignalGroupId apprId, int value);

//    private:
//     std::unordered_map<SignalGroupId, std::unordered_map<SignalGroupId, int>>
//     matrix_;
// };
