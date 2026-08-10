#include "editor.hpp"

#include "document.hpp"
#include "root/editor/graph_editor.hpp"
#include "signal_optimizer/optimizer.hpp"

Editor::Editor(Document& doc, NetworkEventBus& bus)
    : dirty_(),
      doc_(doc),
      ge_(doc.graph(), bus, dirty_),
      de_(doc.demand(), doc.graph(), bus, dirty_),
      se_(doc.signal(), doc.graph(), bus, dirty_, doc.regulations()),
      ce_(doc.control(), doc.signal(), doc.demand(), doc.graph(), bus, dirty_,
          doc.regulations()) {}

std::optional<TimingId> Editor::runOptimizer(Transaction& tx, NodeId at,
                                             int cycleLength) {
    auto result = signal_optimizer::optimize(at, doc_.graph(), doc_.demand(),
                                             cycleLength);
    if (!result) return std::nullopt;

    if (const auto* ng = doc_.signal().data().nodeGroups().get(at)) {
        auto old = *ng;
        for (auto gid : old) se_.removeSignalGroup(tx, gid);
    }

    std::vector<std::vector<SignalGroupId>> keyMap;
    for (const auto& mg : result->mgResults) {
        auto id = se_.createMovementGroup(tx, at, mg.clusters);
        if (!id) return std::nullopt;
        if (!se_.setSgProtection(tx, *id, mg.isProtected))
            return std::nullopt;
        se_.setSgIntervalMinFixed(tx, *id, mg.minGreen);
        keyMap.push_back({*id});
    }
    for (const auto& cg : result->cgResults) {
        std::vector<SignalGroupId> ids;
        for (CrosswalkId cw : cg.crosswalks) {
            auto id = se_.createCrosswalkGroup(tx, at, {cw});
            if (!id) return std::nullopt;
            se_.setSgIntervalMinFixed(tx, *id, cg.minGreen);
            ids.push_back(*id);
        }
        keyMap.push_back(std::move(ids));
    }

    std::vector<StageId> stageIds;
    for (const auto& step : result->stages) {
        std::unordered_set<SignalGroupId> active;
        for (const auto& [key, sa] : step.stage.groups)
            if (sa.isActive)
                active.insert(keyMap[key].begin(), keyMap[key].end());
        auto sid = ce_.createStage(tx, at, std::move(active));
        if (!sid) return std::nullopt;
        stageIds.push_back(*sid);
    }

    const auto n = stageIds.size();
    std::vector<StageStep> steps;
    for (size_t i = 0; i < n; ++i) {
        auto trid =
            ce_.createTransition(tx, at, stageIds[i], stageIds[(i + 1) % n]);
        steps.emplace_back(at, stageIds[i], trid,
                           result->stages[i].stageDuration);
    }

    auto timing = ce_.createTiming(tx, at, "Optimized");
    if (!ce_.setTimingSteps(tx, timing, std::move(steps))) return std::nullopt;
    return timing;
}