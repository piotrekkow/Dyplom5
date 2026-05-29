#pragma once

#include <expected>
#include <vector>

#include "root/editor/errors/graph_errors.hpp"
#include "root/editor/graph_editor.hpp"

class EditorContext;
class CommandStack;
class Transaction;

class GraphUseCases {
   public:
    GraphUseCases(GraphEditor& editor, EditorContext& ec,
                  CommandStack& cmdStack);

    NodeId createNode(Position center);
    EntryId createEntry(NodeId at, Position position, Vector2 heading);
    ExitId createExit(NodeId at, Position position, Vector2 heading);
    CrosswalkSeriesId createCrosswalkSeries(NodeId at);
    CrosswalkId createCrosswalk(CrosswalkSeriesId parent, Line line);

    void setMovementPathSpec(MovementId id, MovementPathSpec spec);
    std::expected<void, MovementSplitError> setMovementSplits(
        MovementId id, std::vector<int> splits);
    void setMovementType(MovementId id, MovementType type);

    void setCrosswalkSeriesPedPath(CrosswalkSeriesId id, Polyline path);
    void setCrosswalkType(CrosswalkId id, CrosswalkType type);
    void setCrosswalkLine(CrosswalkId id, Line line);
    void setCrosswalkEdge(CrosswalkId id, Crosswalk::SegmentParams edge);
    void setCrosswalkSplit(CrosswalkId id, Crosswalk::SegmentParams split);

    std::expected<std::vector<MovementId>, MovementLayoutError> setEntryLayout(
        EntryId id, std::vector<GraphEditor::MovementSpec> specs,
        std::vector<bool> shared);
    void setEntryLanes(EntryId id, std::vector<EntryLaneSpec> specs);
    void setExitLanes(ExitId id, std::vector<ExitLaneSpec> specs);
    void removeEntryLayout(EntryId id);

    void removeNode(NodeId id);
    void removeCrosswalk(CrosswalkId id);
    void removeCrosswalkSeries(CrosswalkSeriesId id);
    void removeEntry(EntryId id);
    void removeExit(ExitId id);

    void setStreamPermittedOverride(NodeId node, StreamId evac, StreamId appr,
                                    bool permitted);
    void eraseStreamPermittedOverride(NodeId node, StreamId evac,
                                      StreamId appr);

    void setStreamIntergreenOverride(NodeId node, StreamId evac, StreamId appr,
                                     int t_maxEvAp, int t_maxApEv);
    void eraseStreamIntergreenOverride(NodeId node, StreamId evac,
                                       StreamId appr);

   private:
    GraphEditor& editor_;
    EditorContext& ec_;
    CommandStack& cmdStack_;
};
