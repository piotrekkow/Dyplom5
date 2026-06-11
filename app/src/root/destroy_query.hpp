#pragma once

#include <road_network/util/ids.hpp>
#include <unordered_set>

class Document;
class SignalEditor;

// Read-only "what would this destroy" query backing confirm dialogs. Mirrors
// the cross-domain cascades performed by EditorContext, but performs no
// mutation. Reuses SignalEditor's const reference-finding queries.
class DestroyQuery {
   public:
    DestroyQuery(const Document& doc, const SignalEditor& signal);

    struct Impact {
        int flows = 0;  // movements carrying non-default demand data
        int signalGroups = 0;
        int timings = 0;
        int configs = 0;
        int intersections = 0;
        [[nodiscard]] bool any() const {
            return flows || signalGroups || timings || configs ||
                   intersections;
        }
    };

    [[nodiscard]] Impact removeNode(NodeId id) const;
    [[nodiscard]] Impact removeEntry(EntryId id) const;
    [[nodiscard]] Impact removeExit(ExitId id) const;
    [[nodiscard]] Impact setEntryLanes(EntryId id) const;
    [[nodiscard]] Impact setExitLanes(ExitId id) const;
    [[nodiscard]] Impact setEntryLayout(EntryId id) const;
    [[nodiscard]] Impact removeCrosswalk(CrosswalkId id) const;
    [[nodiscard]] Impact removeCrosswalkSeries(CrosswalkSeriesId id) const;
    [[nodiscard]] Impact removeSignalGroup(SignalGroupId id) const;
    [[nodiscard]] Impact removeSignalConfig(SignalConfigId id) const;
    [[nodiscard]] Impact removeIntersection(IntersectionId id) const;

   private:
    const Document& doc_;
    const SignalEditor& signal_;

    // sets avoid double counting across overlapping cascades
    struct Acc {
        std::unordered_set<MovementId> movements;
        std::unordered_set<SignalGroupId> groups;
        std::unordered_set<TimingId> timings;
        std::unordered_set<SignalConfigId> configs;
        std::unordered_set<IntersectionId> intersections;
    };
    void accEntryLayout(Acc& acc, EntryId id) const;
    void accNodeLayouts(Acc& acc, NodeId id) const;
    void accCrosswalk(Acc& acc, CrosswalkId id) const;
    void accConfig(Acc& acc, SignalConfigId id) const;
    void accIntersection(Acc& acc, IntersectionId id) const;
    [[nodiscard]] Impact toImpact(const Acc& acc) const;
};
