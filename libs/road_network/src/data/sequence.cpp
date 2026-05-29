#include "road_network/signal/data/sequence.hpp"

#include <algorithm>
#include <expected>

#include "road_network/signal/signal_errors.hpp"

std::expected<void, TimingWarning> SignalSequence::setMinIntervalLength(
    int min) {
    minIntervalLength_ = min;
    if (std::ranges::any_of(
            intervals_, [&](const auto& i) { return i.greenDuration() < min; }))
        return std::unexpected(TimingWarning::UpdatedMinIntervalViolation);
}

std::expected<void, TimingError> SignalSequence::setIntervals(
    std::vector<Interval> intervals) {
    if (std::ranges::any_of(intervals, [&](const auto& i) {
            return i.greenDuration() < minIntervalLength_;
        }))
        return std::unexpected(TimingError::IntervalMinViolation);
    intervals_ = std::move(intervals);
    return {};
}