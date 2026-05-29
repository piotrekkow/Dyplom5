#pragma once

#include "road_network/util/ids.hpp"
#include "sequence.hpp"

class Timing {
   public:
    Timing(NodeId at, int cycleLength) : at_(at), cycleLength_(cycleLength) {}

    [[nodiscard]] const Sequence& sequence(SignalGroupId sg) const {
        return sequences_.at(sg);
    }
    [[nodiscard]] int cycleLength() const { return cycleLength_; }
    [[nodiscard]] NodeId at() const { return at_; }

    std::expected<void, TimingError> setCycleLength(int t);
    void createNewSequence(SignalGroupId sg, int minIntervalLength);
    std::expected<void, TimingWarning> setSgMinInterval(SignalGroupId id,
                                                        int minIntervalLength);
    std::expected<void, TimingError> setIntervals(
        SignalGroupId sg, std::vector<Interval> intervals);

    [[nodiscard]] bool hasSequence(SignalGroupId sg) const {
        return sequences_.contains(sg);
    }
    void removeSequence(SignalGroupId sg) { sequences_.erase(sg); }
    void insertSequence(SignalGroupId sg, Sequence seq) {
        sequences_[sg] = std::move(seq);
    }
    void setAlwaysActive(SignalGroupId sg) {
        sequences_.at(sg) = AlwaysActive{};
    }

   private:
    NodeId at_;
    std::unordered_map<SignalGroupId, Sequence> sequences_{};
    int cycleLength_;
};