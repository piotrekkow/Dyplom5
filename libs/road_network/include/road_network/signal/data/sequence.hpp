#pragma once

#include <expected>
#include <variant>
#include <vector>

#include "../signal_errors.hpp"
#include "interval.hpp"

class SignalSequence {
   public:
    SignalSequence(int minIntervalLength)
        : minIntervalLength_(minIntervalLength) {}

    [[nodiscard]] const std::vector<Interval>& intervals() const {
        return intervals_;
    }
    [[nodiscard]] int minIntervalLength() const { return minIntervalLength_; }

    std::expected<void, TimingWarning> setMinIntervalLength(int min);
    std::expected<void, TimingError> setIntervals(
        std::vector<Interval> intervals);

   private:
    std::vector<Interval> intervals_;
    int minIntervalLength_;
};

// signals of this group always display active signal aspect
struct AlwaysActive {};

using Sequence = std::variant<AlwaysActive, SignalSequence>;