#include "road_network/control.hpp"

#include <algorithm>
#include <cassert>
#include <variant>

#include "road_network/control/data/sequence.hpp"

namespace {

void registerTiming(ControlData& data, SignalConfigId configId, TimingId id) {
    if (!data.configControls().get(configId))
        data.configControls().set(configId, {});
    data.configControls().get(configId)->timings.push_back(id);
}

void unregisterTiming(ControlData& data, SignalConfigId configId, TimingId id) {
    if (auto* cc = data.configControls().get(configId)) {
        auto& ts = cc->timings;
        ts.erase(std::ranges::find(ts, id));
    }
}

}  // namespace

//
//  TIMING LIFECYCLE
//

Control::EntityMutation<TimingId> Control::createTiming(SignalConfigId at,
                                                        int cycleLength) {
    auto id = data_.timings().emplace(at, cycleLength);
    registerTiming(data_, at, id);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.timings().release(removeTimingImpl(id));
            }};
}

Control::ConfigMutation Control::removeTiming(TimingId id) {
    assert(data_.timings().alive(id));
    auto vs = removeTimingImpl(id);
    SignalConfigId configId = vs.value.at();
    return {.id = configId, .undo = [this, vs = std::move(vs)] mutable {
                createTimingImpl(std::move(vs));
            }};
}

//
//  TIMING SEQUENCE MUTATIONS
//

Control::TimingMutation Control::createSequence(TimingId id, SignalGroupId sg,
                                                int minIntervalLength) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();
    timing->createNewSequence(sg, minIntervalLength);
    return {.timingId = id,
            .sgId = sg,
            .at = configId,
            .undo = [this, id, sg] {
                data_.timings().get(id)->removeSequence(sg);
            }};
}

Control::TimingMutation Control::removeSequence(TimingId id, SignalGroupId sg) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();
    Sequence snap = timing->sequence(sg);
    timing->removeSequence(sg);
    return {.timingId = id,
            .sgId = sg,
            .at = configId,
            .undo = [this, id, sg, snap = std::move(snap)] mutable {
                data_.timings().get(id)->insertSequence(sg, std::move(snap));
            }};
}

Control::TimingMutation Control::setAlwaysActive(TimingId id,
                                                 SignalGroupId sg) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();
    Sequence snap = timing->sequence(sg);
    timing->setAlwaysActive(sg);
    return {.timingId = id,
            .sgId = sg,
            .at = configId,
            .undo = [this, id, sg, snap = std::move(snap)] mutable {
                data_.timings().get(id)->insertSequence(sg, std::move(snap));
            }};
}

std::expected<Control::TimingMutation, TimingError> Control::setIntervals(
    TimingId id, SignalGroupId sg, std::vector<Interval> intervals) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();

    std::vector<Interval> old;
    if (const auto* ss = std::get_if<SignalSequence>(&timing->sequence(sg)))
        old = ss->intervals();

    auto res = timing->setIntervals(sg, std::move(intervals));
    if (!res) return std::unexpected(res.error());

    return TimingMutation{.timingId = id,
                          .sgId = sg,
                          .at = configId,
                          .undo = [this, id, sg, old = std::move(old)] mutable {
                              assert(data_.timings()
                                         .get(id)
                                         ->setIntervals(sg, std::move(old))
                                         .has_value());
                          }};
}

std::expected<Control::TimingMutation, TimingWarning>
Control::setMinIntervalLength(TimingId id, SignalGroupId sg, int min) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();

    int old = 0;
    if (const auto* ss = std::get_if<SignalSequence>(&timing->sequence(sg)))
        old = ss->minIntervalLength();

    auto res = timing->setSgMinInterval(sg, min);
    if (!res) return std::unexpected(res.error());

    return TimingMutation{
        .timingId = id,
        .sgId = sg,
        .at = configId,
        .undo = [this, id, sg, old] {
            assert(
                data_.timings().get(id)->setSgMinInterval(sg, old).has_value());
        }};
}

//
//  IMPLEMENTATION HELPERS
//

void Control::createTimingImpl(SlotMap<TimingTag, Timing>::VacatedSlot vs) {
    SignalConfigId configId = vs.value.at();
    registerTiming(data_, configId, vs.id);
    data_.timings().reinsert(std::move(vs));
}

SlotMap<TimingTag, Timing>::VacatedSlot Control::removeTimingImpl(TimingId id) {
    auto vs = data_.timings().vacate(id);
    SignalConfigId configId = vs.value.at();
    unregisterTiming(data_, configId, id);
    return vs;
}
