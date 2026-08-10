#include "log.hpp"

#include <print>
#include <road_network/graph.hpp>
#include <road_network/signal.hpp>

#include "print_matrix.hpp"

namespace logs {
namespace {
std::string streamLabel(StreamId id) {
    return std::visit(
        [](auto id) {
            constexpr char prefix =
                std::is_same_v<decltype(id), MovementId> ? 'm' : 'c';
            return std::format("{}{}", prefix, id.index());
        },
        id);
}

std::vector<StreamId> streamsAt(NodeId node, const Graph& graph) {
    const auto* n = graph.data().nodes().get(node);
    if (!n) return {};

    std::vector<StreamId> streams;
    for (EntryId eid : n->entries()) {
        const auto* entry = graph.data().entries().get(eid);
        if (!entry) continue;
        for (MovementId mid : entry->movements()) streams.emplace_back(mid);
    }
    for (CrosswalkSeriesId csid : n->crosswalkSeries()) {
        const auto* cs = graph.data().crosswalkSeries().get(csid);
        if (!cs) continue;
        for (CrosswalkId cid : cs->crosswalks()) streams.emplace_back(cid);
    }
    return streams;
}
}  // namespace

void streamPermitted(NodeId node, const Graph& graph) {
    const auto* matrix = graph.derived().streamPermitted().get(node);
    if (!matrix) {
        std::println("[log] no stream permitted matrix at node {}",
                     node.index());
        return;
    }

    const auto streams = streamsAt(node, graph);
    printMatrix(std::format("StreamPermittedMap  node {}", node.index()),
                *matrix, std::span<const StreamId>(streams), streamLabel,
                [](bool permitted) { return permitted ? "1" : "-"; });
}

void streamIntergreen(NodeId node, const Graph& graph) {
    const auto* matrix = graph.derived().streamIntergreens().get(node);
    if (!matrix) {
        std::println("[log] no stream intergreen matrix at node {}",
                     node.index());
        return;
    }

    const auto streams = streamsAt(node, graph);
    printMatrix(std::format("StreamIntergreenMap  node {}", node.index()),
                *matrix, std::span<const StreamId>(streams), streamLabel);
}

void intergreenMatrix(NodeId node, const Signal& signal) {
    const auto* matrix = signal.derived().intergreenMatrices().get(node);
    if (!matrix) {
        std::println("[log] no intergreen matrix at node {}", node.index());
        return;
    }

    const auto* groups = signal.data().nodeGroups().get(node);
    if (!groups) return;

    auto label = [&](SignalGroupId id) {
        const SignalGroup* g = signal.data().signalGroups().get(id);
        const std::string suffix =
            g && std::holds_alternative<MovementSignalGroup>(*g) ? "K" : "PR";
        return std::format("{}{}", id.index(), suffix);
    };
    printMatrix(std::format("IntergreenMatrix  node {}", node.index()), *matrix,
                std::span<const SignalGroupId>(*groups), label);
}
}  // namespace logs
