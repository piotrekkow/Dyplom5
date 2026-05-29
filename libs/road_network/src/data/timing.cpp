#include "road_network/signal/data/timing.hpp"

#include <algorithm>
#include <expected>
#include <variant>

#include "road_network/signal/data/sequence.hpp"
#include "road_network/signal/signal_errors.hpp"

namespace {
bool intervalsFitCycle(const std::vector<Interval>& intervals,
                       int cycleLength) {
    if (intervals.empty()) return true;
    std::vector<std::pair<int, int>> occupied;
    for (const auto& interval : intervals) {
        int len = interval.totalDuration();
        if (len > cycleLength) return false;
        int s = ((interval.intervalStart() % cycleLength) + cycleLength) %
                cycleLength;
        occupied.emplace_back(s, s + len);
    }
    std::ranges::sort(occupied);  // by first element, ascending

    for (size_t i = 0; i + 1 < occupied.size(); ++i) {
        if (occupied[i].second > occupied[i + 1].first) return false;
    }
    if (occupied.back().second > cycleLength) {
        int wrappedEnd = occupied.back().second - cycleLength;
        if (wrappedEnd > occupied.front().first) return false;
    }
    return true;
}
}  // namespace

std::expected<void, TimingError> Timing::setCycleLength(int t) {
    if (!std::ranges::all_of(sequences_, [&](const auto& entry) {
            const auto& seq = entry.second;
            if (auto* ss = std::get_if<SignalSequence>(&seq))
                return intervalsFitCycle(ss->intervals(), t);
            // else if AlwaysActive
            return true;
        })) {
        return std::unexpected(TimingError::IntervalsWontFitInCycle);
    }
    cycleLength_ = t;
    return {};
}

void Timing::createNewSequence(SignalGroupId sg, int minIntervalLength) {
    sequences_.emplace(sg, SignalSequence(minIntervalLength));
}

std::expected<void, TimingWarning> Timing::setSgMinInterval(
    SignalGroupId id, int minIntervalLength) {
    if (auto* ss = std::get_if<SignalSequence>(&sequences_.at(id)))
        return ss->setMinIntervalLength(minIntervalLength);
    return {};
}

std::expected<void, TimingError> Timing::setIntervals(
    SignalGroupId sg, std::vector<Interval> intervals) {
    if (!intervalsFitCycle(intervals, cycleLength_))
        return std::unexpected(TimingError::IntervalsWontFitInCycle);
    if (auto* ss = std::get_if<SignalSequence>(&sequences_.at(sg))) {
        return ss->setIntervals(std::move(intervals));
    }
    return std::unexpected(
        TimingError::CannotSetIntervalsOnAlwaysActiveSequence);
}