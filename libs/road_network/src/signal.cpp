#include "road_network/signal.hpp"

#include <algorithm>
#include <cassert>
#include <variant>

#include "derived/signal_compute.hpp"
#include "road_network/signal/data/sequence.hpp"
#include "road_network/signal/data/signal_group.hpp"
#include "road_network/signal/data/timing.hpp"

namespace {

void registerSg(SignalData& data, NodeId nodeId, SignalGroupId id) {
    if (!data.nodeSignals().get(nodeId)) data.nodeSignals().set(nodeId, {});
    data.nodeSignals().get(nodeId)->signalGroups.push_back(id);
}

void unregisterSg(SignalData& data, NodeId nodeId, SignalGroupId id) {
    if (auto* ns = data.nodeSignals().get(nodeId)) {
        auto& sgs = ns->signalGroups;
        sgs.erase(std::ranges::find(sgs, id));
    }
}

void registerTiming(SignalData& data, NodeId nodeId, TimingId id) {
    if (!data.nodeSignals().get(nodeId)) data.nodeSignals().set(nodeId, {});
    data.nodeSignals().get(nodeId)->timings.push_back(id);
}

void unregisterTiming(SignalData& data, NodeId nodeId, TimingId id) {
    if (auto* ns = data.nodeSignals().get(nodeId)) {
        auto& ts = ns->timings;
        ts.erase(std::ranges::find(ts, id));
    }
}

}  // namespace

//
//  SIGNAL GROUP
//

Signal::EntityMutation<SignalGroupId> Signal::createSignalGroup(
    SignalGroup sg) {
    NodeId nodeId = std::visit([](const auto& x) { return x.at(); }, sg);
    auto id = data_.signalGroups().insert(std::move(sg));
    registerSg(data_, nodeId, id);
    dirty_.insert(nodeId);
    return {.id = id, .at = nodeId, .undo = [this, id] {
                data_.signalGroups().release(removeSignalGroupImpl(id));
            }};
}

Signal::EntityMutation<SignalGroupId> Signal::createMovementGroup(
    NodeId at, std::vector<ClusterId> clusters,
    std::vector<MovementId> movements, bool isProtected, int intervalMin) {
    return createSignalGroup(MovementSignalGroup{at, std::move(clusters),
                                                 std::move(movements),
                                                 isProtected, intervalMin});
}

Signal::EntityMutation<SignalGroupId> Signal::createCrosswalkGroup(
    NodeId at, std::vector<CrosswalkId> crosswalks, int intervalMin) {
    return createSignalGroup(
        CrosswalkSignalGroup{at, std::move(crosswalks), intervalMin});
}

Signal::EntityMutation<SignalGroupId> Signal::createTurnArrowGroup(
    NodeId at, MovementId movement, int intervalMin) {
    return createSignalGroup(
        ConditionalTurnArrowSignalGroup{at, movement, intervalMin});
}

Signal::EntityMutation<SignalGroupId> Signal::setIntervalMin(SignalGroupId id,
                                                             int min) {
    assert(data_.signalGroups().alive(id));
    auto* sg = data_.signalGroups().get(id);
    NodeId nodeId = std::visit([](const auto& x) { return x.at(); }, *sg);
    int old = std::visit([](const auto& x) { return x.intervalMin(); }, *sg);
    auto apply = [this, id, nodeId](int v) {
        std::visit([v](auto& x) { x.setIntervalMin(v); },
                   *data_.signalGroups().get(id));
        dirty_.insert(nodeId);
    };
    apply(min);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}

Signal::EntityMutation<SignalGroupId> Signal::setProtected(SignalGroupId id,
                                                           bool isProtected) {
    assert(data_.signalGroups().alive(id));
    auto* sg = data_.signalGroups().get(id);
    NodeId nodeId = std::visit([](const auto& x) { return x.at(); }, *sg);
    bool old = std::get<MovementSignalGroup>(*sg).isProtected();
    auto apply = [this, id, nodeId](bool v) {
        std::get<MovementSignalGroup>(*data_.signalGroups().get(id))
            .setProtected(v);
        dirty_.insert(nodeId);
    };
    apply(isProtected);
    return {.id = id, .at = nodeId, .undo = [apply, old] { apply(old); }};
}

Signal::NodeMutation Signal::removeSignalGroup(SignalGroupId id) {
    assert(data_.signalGroups().alive(id));
    auto vs = removeSignalGroupImpl(id);
    NodeId nodeId = std::visit([](const auto& x) { return x.at(); }, vs.value);
    return {.id = nodeId, .undo = [this, vs = std::move(vs)] mutable {
                createSignalGroupImpl(std::move(vs));
            }};
}

//
//  TIMING LIFECYCLE
//

Signal::EntityMutation<TimingId> Signal::createTiming(NodeId at,
                                                      int cycleLength) {
    auto id = data_.timings().emplace(at, cycleLength);
    registerTiming(data_, at, id);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.timings().release(removeTimingImpl(id));
            }};
}

Signal::NodeMutation Signal::removeTiming(TimingId id) {
    assert(data_.timings().alive(id));
    auto vs = removeTimingImpl(id);
    NodeId nodeId = vs.value.at();
    return {.id = nodeId, .undo = [this, vs = std::move(vs)] mutable {
                createTimingImpl(std::move(vs));
            }};
}

//
//  TIMING SEQUENCE MUTATIONS
//

Signal::TimingMutation Signal::createSequence(TimingId id, SignalGroupId sg,
                                              int minIntervalLength) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    NodeId nodeId = timing->at();
    timing->createNewSequence(sg, minIntervalLength);
    return {.timingId = id, .sgId = sg, .at = nodeId, .undo = [this, id, sg] {
                data_.timings().get(id)->removeSequence(sg);
            }};
}

Signal::TimingMutation Signal::removeSequence(TimingId id, SignalGroupId sg) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    NodeId nodeId = timing->at();
    Sequence snap = timing->sequence(sg);
    timing->removeSequence(sg);
    return {.timingId = id,
            .sgId = sg,
            .at = nodeId,
            .undo = [this, id, sg, snap = std::move(snap)] mutable {
                data_.timings().get(id)->insertSequence(sg, std::move(snap));
            }};
}

Signal::TimingMutation Signal::setAlwaysActive(TimingId id, SignalGroupId sg) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    NodeId nodeId = timing->at();
    Sequence snap = timing->sequence(sg);
    timing->setAlwaysActive(sg);
    return {.timingId = id,
            .sgId = sg,
            .at = nodeId,
            .undo = [this, id, sg, snap = std::move(snap)] mutable {
                data_.timings().get(id)->insertSequence(sg, std::move(snap));
            }};
}

std::expected<Signal::TimingMutation, TimingError> Signal::setIntervals(
    TimingId id, SignalGroupId sg, std::vector<Interval> intervals) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    NodeId nodeId = timing->at();

    std::vector<Interval> old;
    if (const auto* ss = std::get_if<SignalSequence>(&timing->sequence(sg)))
        old = ss->intervals();

    auto res = timing->setIntervals(sg, std::move(intervals));
    if (!res) return std::unexpected(res.error());

    return TimingMutation{.timingId = id,
                          .sgId = sg,
                          .at = nodeId,
                          .undo = [this, id, sg, old = std::move(old)] mutable {
                              assert(data_.timings()
                                         .get(id)
                                         ->setIntervals(sg, std::move(old))
                                         .has_value());
                          }};
}

std::expected<Signal::TimingMutation, TimingWarning>
Signal::setMinIntervalLength(TimingId id, SignalGroupId sg, int min) {
    assert(data_.timings().alive(id));
    auto* timing = data_.timings().get(id);
    NodeId nodeId = timing->at();

    int old = 0;
    if (const auto* ss = std::get_if<SignalSequence>(&timing->sequence(sg)))
        old = ss->minIntervalLength();

    auto res = timing->setSgMinInterval(sg, min);
    if (!res) return std::unexpected(res.error());

    return TimingMutation{
        .timingId = id, .sgId = sg, .at = nodeId, .undo = [this, id, sg, old] {
            assert(
                data_.timings().get(id)->setSgMinInterval(sg, old).has_value());
        }};
}

//
//  DERIVED
//

void Signal::recomputeDerived(const GraphDerived& graphDerived, NodeId id) {
    derived_.intergreenMatrix().set(
        id, signal_compute::intergreenMatrix(graphDerived, data_, id));
}

//
//  IMPLEMENTATION HELPERS
//

void Signal::createSignalGroupImpl(
    SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot vs) {
    NodeId nodeId = std::visit([](const auto& x) { return x.at(); }, vs.value);
    registerSg(data_, nodeId, vs.id);
    data_.signalGroups().reinsert(std::move(vs));
    dirty_.insert(nodeId);
}

SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot Signal::removeSignalGroupImpl(
    SignalGroupId id) {
    auto vs = data_.signalGroups().vacate(id);
    NodeId nodeId = std::visit([](const auto& x) { return x.at(); }, vs.value);
    unregisterSg(data_, nodeId, id);
    dirty_.insert(nodeId);
    return vs;
}

void Signal::createTimingImpl(SlotMap<TimingTag, Timing>::VacatedSlot vs) {
    NodeId nodeId = vs.value.at();
    registerTiming(data_, nodeId, vs.id);
    data_.timings().reinsert(std::move(vs));
}

SlotMap<TimingTag, Timing>::VacatedSlot Signal::removeTimingImpl(TimingId id) {
    auto vs = data_.timings().vacate(id);
    NodeId nodeId = vs.value.at();
    unregisterTiming(data_, nodeId, id);
    return vs;
}
