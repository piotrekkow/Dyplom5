#include "road_network/signal.hpp"

#include <algorithm>
#include <cassert>
#include <variant>

#include "derived/signal_compute.hpp"
#include "road_network/signal/data/signal_group.hpp"

namespace {

void registerConfig(SignalData& data, NodeId nodeId, SignalConfigId id) {
    if (!data.nodeSignals().get(nodeId)) data.nodeSignals().set(nodeId, {});
    data.nodeSignals().get(nodeId)->configs.push_back(id);
}

void unregisterConfig(SignalData& data, NodeId nodeId, SignalConfigId id) {
    if (auto* ns = data.nodeSignals().get(nodeId)) {
        auto& cfgs = ns->configs;
        cfgs.erase(std::ranges::find(cfgs, id));
    }
}

void registerSg(SignalData& data, SignalConfigId configId, SignalGroupId id) {
    data.configs().get(configId)->signalGroups().push_back(id);
}

void unregisterSg(SignalData& data, SignalConfigId configId, SignalGroupId id) {
    auto& sgs = data.configs().get(configId)->signalGroups();
    sgs.erase(std::ranges::find(sgs, id));
}

}  // namespace

//
//  CONFIG LIFECYCLE
//

Signal::ConfigMutation Signal::createConfig(NodeId at, std::string name) {
    auto id = data_.configs().emplace(at, std::move(name));
    registerConfig(data_, at, id);
    return {.id = id, .at = at, .undo = [this, id] {
                data_.configs().release(removeConfigImpl(id));
            }};
}

Signal::ConfigMutation Signal::removeConfig(SignalConfigId id) {
    assert(data_.configs().alive(id));
    assert(data_.configs().get(id)->signalGroups().empty() &&
           "remove a config's signal groups before removing the config");
    auto vs = removeConfigImpl(id);
    NodeId nodeId = vs.value.at();
    return {.id = id, .at = nodeId, .undo = [this, vs = std::move(vs)] mutable {
                createConfigImpl(std::move(vs));
            }};
}

Signal::ConfigMutation Signal::renameConfig(SignalConfigId id,
                                            std::string name) {
    assert(data_.configs().alive(id));
    auto* cfg = data_.configs().get(id);
    NodeId nodeId = cfg->at();
    std::string old = cfg->name();
    cfg->setName(std::move(name));
    return {.id = id, .at = nodeId, .undo = [this, id, old = std::move(old)] mutable {
                data_.configs().get(id)->setName(std::move(old));
            }};
}

//
//  SIGNAL GROUP
//

Signal::GroupMutation Signal::createSignalGroup(SignalGroup sg) {
    SignalConfigId configId =
        std::visit([](const auto& x) { return x.config(); }, sg);
    NodeId nodeId = data_.configs().get(configId)->at();
    auto id = data_.signalGroups().insert(std::move(sg));
    registerSg(data_, configId, id);
    dirty_.insert(configId);
    return {.id = id, .config = configId, .at = nodeId, .undo = [this, id] {
                data_.signalGroups().release(removeSignalGroupImpl(id));
            }};
}

Signal::GroupMutation Signal::createMovementGroup(
    SignalConfigId config, std::vector<ClusterId> clusters,
    std::vector<MovementId> movements, bool isProtected, int intervalMin) {
    return createSignalGroup(MovementSignalGroup{config, std::move(clusters),
                                                 std::move(movements),
                                                 isProtected, intervalMin});
}

Signal::GroupMutation Signal::createCrosswalkGroup(
    SignalConfigId config, std::vector<CrosswalkId> crosswalks,
    int intervalMin) {
    return createSignalGroup(
        CrosswalkSignalGroup{config, std::move(crosswalks), intervalMin});
}

Signal::GroupMutation Signal::createTurnArrowGroup(SignalConfigId config,
                                                   MovementId movement,
                                                   int intervalMin) {
    return createSignalGroup(
        ConditionalTurnArrowSignalGroup{config, movement, intervalMin});
}

Signal::GroupMutation Signal::setIntervalMin(SignalGroupId id, int min) {
    assert(data_.signalGroups().alive(id));
    auto* sg = data_.signalGroups().get(id);
    SignalConfigId configId =
        std::visit([](const auto& x) { return x.config(); }, *sg);
    NodeId nodeId = data_.configs().get(configId)->at();
    int old = std::visit([](const auto& x) { return x.intervalMin(); }, *sg);
    auto apply = [this, id, configId](int v) {
        std::visit([v](auto& x) { x.setIntervalMin(v); },
                   *data_.signalGroups().get(id));
        dirty_.insert(configId);
    };
    apply(min);
    return {.id = id,
            .config = configId,
            .at = nodeId,
            .undo = [apply, old] { apply(old); }};
}

Signal::GroupMutation Signal::setProtected(SignalGroupId id,
                                           bool isProtected) {
    assert(data_.signalGroups().alive(id));
    auto* sg = data_.signalGroups().get(id);
    SignalConfigId configId =
        std::visit([](const auto& x) { return x.config(); }, *sg);
    NodeId nodeId = data_.configs().get(configId)->at();
    bool old = std::get<MovementSignalGroup>(*sg).isProtected();
    auto apply = [this, id, configId](bool v) {
        std::get<MovementSignalGroup>(*data_.signalGroups().get(id))
            .setProtected(v);
        dirty_.insert(configId);
    };
    apply(isProtected);
    return {.id = id,
            .config = configId,
            .at = nodeId,
            .undo = [apply, old] { apply(old); }};
}

Signal::GroupMutation Signal::removeSignalGroup(SignalGroupId id) {
    assert(data_.signalGroups().alive(id));
    auto vs = removeSignalGroupImpl(id);
    SignalConfigId configId =
        std::visit([](const auto& x) { return x.config(); }, vs.value);
    NodeId nodeId = data_.configs().get(configId)->at();
    return {.id = id,
            .config = configId,
            .at = nodeId,
            .undo = [this, vs = std::move(vs)] mutable {
                createSignalGroupImpl(std::move(vs));
            }};
}

//
//  DERIVED
//

void Signal::recomputeDerived(const GraphDerived& graphDerived,
                              SignalConfigId id) {
    if (!data_.configs().alive(id)) {
        derived_.intergreenMatrix().erase(id);
        return;
    }
    derived_.intergreenMatrix().set(
        id, signal_compute::intergreenMatrix(graphDerived, data_, id));
}

//
//  IMPLEMENTATION HELPERS
//

void Signal::createConfigImpl(
    SlotMap<SignalConfigTag, SignalConfig>::VacatedSlot vs) {
    NodeId nodeId = vs.value.at();
    registerConfig(data_, nodeId, vs.id);
    data_.configs().reinsert(std::move(vs));
}

SlotMap<SignalConfigTag, SignalConfig>::VacatedSlot Signal::removeConfigImpl(
    SignalConfigId id) {
    auto vs = data_.configs().vacate(id);
    NodeId nodeId = vs.value.at();
    unregisterConfig(data_, nodeId, id);
    dirty_.insert(id);
    return vs;
}

void Signal::createSignalGroupImpl(
    SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot vs) {
    SignalConfigId configId =
        std::visit([](const auto& x) { return x.config(); }, vs.value);
    registerSg(data_, configId, vs.id);
    data_.signalGroups().reinsert(std::move(vs));
    dirty_.insert(configId);
}

SlotMap<SignalGroupTag, SignalGroup>::VacatedSlot Signal::removeSignalGroupImpl(
    SignalGroupId id) {
    auto vs = data_.signalGroups().vacate(id);
    SignalConfigId configId =
        std::visit([](const auto& x) { return x.config(); }, vs.value);
    unregisterSg(data_, configId, id);
    dirty_.insert(configId);
    return vs;
}
