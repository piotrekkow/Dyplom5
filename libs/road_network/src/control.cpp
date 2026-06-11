#include "road_network/control.hpp"

#include <algorithm>
#include <cassert>
#include <variant>

#include "derived/control_compute.hpp"
#include "road_network/control/data/sequence.hpp"
#include "road_network/signal/signal_data.hpp"

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
    dirty_.insert(at);
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

Control::TimingMutation Control::createSequence(TimingId id, SignalGroupId sg) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();
    timing->createNewSequence(sg);
    dirty_.insert(configId);
    return {.timingId = id,
            .sgId = sg,
            .at = configId,
            .undo = [this, id, sg, configId] {
                data_.timings().get(id)->removeSequence(sg);
                dirty_.insert(configId);
            }};
}

Control::TimingMutation Control::removeSequence(TimingId id, SignalGroupId sg) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();
    Sequence snap = timing->sequence(sg);
    timing->removeSequence(sg);
    dirty_.insert(configId);
    return {.timingId = id,
            .sgId = sg,
            .at = configId,
            .undo = [this, id, sg, configId, snap = std::move(snap)] mutable {
                data_.timings().get(id)->insertSequence(sg, std::move(snap));
                dirty_.insert(configId);
            }};
}

Control::TimingMutation Control::setAlwaysActive(TimingId id,
                                                 SignalGroupId sg) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    SignalConfigId configId = timing->at();
    Sequence snap = timing->sequence(sg);
    timing->setAlwaysActive(sg);
    dirty_.insert(configId);
    return {.timingId = id,
            .sgId = sg,
            .at = configId,
            .undo = [this, id, sg, configId, snap = std::move(snap)] mutable {
                data_.timings().get(id)->insertSequence(sg, std::move(snap));
                dirty_.insert(configId);
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
    dirty_.insert(configId);

    return TimingMutation{
        .timingId = id,
        .sgId = sg,
        .at = configId,
        .undo = [this, id, sg, configId, old = std::move(old)] mutable {
            assert(data_.timings()
                       .get(id)
                       ->setIntervals(sg, std::move(old))
                       .has_value());
            dirty_.insert(configId);
        }};
}

//
//  DERIVED
//

void Control::solve(const GraphData& graph, const SignalData& signal,
                    const SignalDerived& signalDerived) {
    for (SignalConfigId id : dirty_)
        recomputeDerived(graph, signal, signalDerived, id);
    dirty_.clear();
}

void Control::recomputeDerived(const GraphData& graph, const SignalData& signal,
                               const SignalDerived& signalDerived,
                               SignalConfigId id) {
    if (!signal.configs().alive(id)) {
        derived_.configCompleteness().erase(id);
        return;
    }
    derived_.configCompleteness().set(
        id, control_compute::configCompleteness(graph, signal, id));

    if (const auto* cc = data_.configControls().get(id))
        for (TimingId tId : cc->timings)
            derived_.timingValidity().set(
                tId, control_compute::timingValidity(
                         signal, signalDerived, *data_.timings().get(tId)));
}

//
//  IMPLEMENTATION HELPERS
//

void Control::createTimingImpl(SlotMap<TimingTag, Timing>::VacatedSlot vs) {
    SignalConfigId configId = vs.value.at();
    registerTiming(data_, configId, vs.id);
    data_.timings().reinsert(std::move(vs));
    dirty_.insert(configId);
}

SlotMap<TimingTag, Timing>::VacatedSlot Control::removeTimingImpl(TimingId id) {
    auto vs = data_.timings().vacate(id);
    SignalConfigId configId = vs.value.at();
    unregisterTiming(data_, configId, id);
    derived_.timingValidity().erase(id);
    dirty_.insert(configId);
    return vs;
}
