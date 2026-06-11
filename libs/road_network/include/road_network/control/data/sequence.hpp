#pragma once

#include <variant>
#include <vector>

#include "interval.hpp"

// Interval-min limits live on the SignalGroup (signal domain); violations are
// flagged by the control validation pass instead of being rejected here.
class SignalSequence {
   public:
    [[nodiscard]] const std::vector<Interval>& intervals() const {
        return intervals_;
    }

    void setIntervals(std::vector<Interval> intervals) {
        intervals_ = std::move(intervals);
    }

   private:
    std::vector<Interval> intervals_;
};

// signals of this group always display active signal aspect
struct AlwaysActive {};

using Sequence = std::variant<AlwaysActive, SignalSequence>;
