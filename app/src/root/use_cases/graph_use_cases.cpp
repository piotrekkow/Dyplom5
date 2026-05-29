#include "graph_use_cases.hpp"

#include "root/command/transaction.hpp"
#include "root/command_stack.hpp"
#include "root/editor_context.hpp"

GraphUseCases::GraphUseCases(GraphEditor& editor, EditorContext& ec,
                             CommandStack& cmdStack)
    : editor_(editor), ec_(ec), cmdStack_(cmdStack) {}

NodeId GraphUseCases::createNode(Position center) {
    Transaction tx;
    auto id = editor_.createNodeTx(tx, center);
    cmdStack_.push(tx.commit());
    return id;
}

EntryId GraphUseCases::createEntry(NodeId at, Position position,
                                   Vector2 heading) {
    Transaction tx;
    auto id = editor_.createEntryTx(tx, at, position, heading);
    cmdStack_.push(tx.commit());
    return id;
}

ExitId GraphUseCases::createExit(NodeId at, Position position,
                                 Vector2 heading) {
    Transaction tx;
    auto id = editor_.createExitTx(tx, at, position, heading);
    cmdStack_.push(tx.commit());
    return id;
}

CrosswalkSeriesId GraphUseCases::createCrosswalkSeries(NodeId at) {
    Transaction tx;
    auto id = editor_.createCrosswalkSeriesTx(tx, at);
    cmdStack_.push(tx.commit());
    return id;
}

CrosswalkId GraphUseCases::createCrosswalk(CrosswalkSeriesId parent,
                                           Line line) {
    Transaction tx;
    auto id = editor_.createCrosswalkTx(tx, parent, line);
    cmdStack_.push(tx.commit());
    return id;
}

void GraphUseCases::setMovementPathSpec(MovementId id, MovementPathSpec spec) {
    Transaction tx;
    editor_.setMovementPathSpecTx(tx, id, spec);
    cmdStack_.push(tx.commit());
}

std::expected<void, MovementSplitError> GraphUseCases::setMovementSplits(
    MovementId id, std::vector<int> splits) {
    Transaction tx;
    auto res = editor_.setMovementSplitsTx(tx, id, std::move(splits));
    if (!res) return res;
    cmdStack_.push(tx.commit());
    return {};
}

void GraphUseCases::setMovementType(MovementId id, MovementType type) {
    Transaction tx;
    editor_.setMovementTypeTx(tx, id, type);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setCrosswalkSeriesPedPath(CrosswalkSeriesId id,
                                              Polyline path) {
    Transaction tx;
    editor_.setCrosswalkSeriesPedPathTx(tx, id, std::move(path));
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setCrosswalkType(CrosswalkId id, CrosswalkType type) {
    Transaction tx;
    editor_.setCrosswalkTypeTx(tx, id, type);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setCrosswalkLine(CrosswalkId id, Line line) {
    Transaction tx;
    editor_.setCrosswalkLineTx(tx, id, line);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setCrosswalkEdge(CrosswalkId id,
                                     Crosswalk::SegmentParams edge) {
    Transaction tx;
    editor_.setCrosswalkEdgeTx(tx, id, edge);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setCrosswalkSplit(CrosswalkId id,
                                      Crosswalk::SegmentParams split) {
    Transaction tx;
    editor_.setCrosswalkSplitTx(tx, id, split);
    cmdStack_.push(tx.commit());
}

std::expected<std::vector<MovementId>, MovementLayoutError>
GraphUseCases::setEntryLayout(EntryId id,
                              std::vector<GraphEditor::MovementSpec> specs,
                              std::vector<bool> shared) {
    Transaction tx;
    auto res =
        ec_.setEntryLayoutTx(tx, id, std::move(specs), std::move(shared));
    if (!res) return res;
    cmdStack_.push(tx.commit());
    return res;
}

void GraphUseCases::setEntryLanes(EntryId id,
                                  std::vector<EntryLaneSpec> specs) {
    Transaction tx;
    ec_.setEntryLanesTx(tx, id, std::move(specs));
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setExitLanes(ExitId id, std::vector<ExitLaneSpec> specs) {
    Transaction tx;
    ec_.setExitLanesTx(tx, id, std::move(specs));
    cmdStack_.push(tx.commit());
}

void GraphUseCases::removeEntryLayout(EntryId id) {
    Transaction tx;
    ec_.removeEntryLayoutTx(tx, id);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::removeNode(NodeId id) {
    Transaction tx;
    ec_.removeNodeTx(tx, id);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::removeCrosswalk(CrosswalkId id) {
    Transaction tx;
    ec_.removeCrosswalkTx(tx, id);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::removeCrosswalkSeries(CrosswalkSeriesId id) {
    Transaction tx;
    ec_.removeCrosswalkSeriesTx(tx, id);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::removeEntry(EntryId id) {
    Transaction tx;
    ec_.removeEntryTx(tx, id);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::removeExit(ExitId id) {
    Transaction tx;
    ec_.removeExitTx(tx, id);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setStreamPermittedOverride(NodeId node, StreamId evac,
                                               StreamId appr, bool permitted) {
    Transaction tx;
    editor_.setStreamPermittedOverrideTx(tx, node, evac, appr, permitted);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::eraseStreamPermittedOverride(NodeId node, StreamId evac,
                                                 StreamId appr) {
    Transaction tx;
    editor_.eraseStreamPermittedOverrideTx(tx, node, evac, appr);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::setStreamIntergreenOverride(NodeId node, StreamId evac,
                                                StreamId appr, int t_maxEvAp,
                                                int t_maxApEv) {
    Transaction tx;
    editor_.setStreamIntergreenOverrideTx(tx, node, evac, appr, t_maxEvAp,
                                          t_maxApEv);
    cmdStack_.push(tx.commit());
}

void GraphUseCases::eraseStreamIntergreenOverride(NodeId node, StreamId evac,
                                                  StreamId appr) {
    Transaction tx;
    editor_.eraseStreamIntergreenOverrideTx(tx, node, evac, appr);
    cmdStack_.push(tx.commit());
}
