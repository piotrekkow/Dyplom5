#include "phase_cover.hpp"

#include <algorithm>
#include <limits>

namespace {

static constexpr size_t kMaxCovers = 5000;
// Redundant phases allowed beyond minimal coverage (enables double-service of
// saturated groups such as {458}{5789}{679} where 5K is served twice).
static constexpr int kRedundantPhases = 1;

// DFS cover enumeration using pivot branching.
// Always chooses the uncovered required group reachable by the fewest
// remaining phases as the branching variable — this prunes hard and keeps
// the search focused.
// When uncovered == 0, records the current cover and, if redundantBudget > 0,
// continues to add one more phase at a time so non-minimal covers with
// deliberate redundancy are also emitted.
void enumCoversRec(const std::vector<GPhase>& phases,
                   const std::vector<GId>& sortedReq,
                   int startIdx,
                   int redundantBudget,
                   std::vector<GPhase>& current,
                   std::vector<bool>& covered,
                   int uncovered,
                   std::vector<std::vector<GPhase>>& result) {
    if (result.size() >= kMaxCovers) return;

    if (uncovered == 0) {
        result.push_back(current);
        if (redundantBudget > 0) {
            for (int i = startIdx; i < (int)phases.size(); ++i) {
                if (result.size() >= kMaxCovers) return;
                current.push_back(phases[i]);
                enumCoversRec(phases, sortedReq, i + 1, redundantBudget - 1,
                              current, covered, 0, result);
                current.pop_back();
            }
        }
        return;
    }

    if (startIdx >= (int)phases.size()) return;

    // Pivot: uncovered required group covered by the fewest phases[startIdx:]
    GId pivot = -1;
    int pivotCnt = std::numeric_limits<int>::max();
    for (GId g : sortedReq) {
        if (covered[g]) continue;
        int cnt = 0;
        for (int i = startIdx; i < (int)phases.size(); ++i)
            if (std::ranges::any_of(phases[i].groupIds,
                                    [g](GId x) { return x == g; })) ++cnt;
        if (cnt == 0) return;  // prune: g is unreachable with remaining phases
        if (cnt < pivotCnt) { pivotCnt = cnt; pivot = g; }
    }
    if (pivot < 0) return;

    for (int i = startIdx; i < (int)phases.size(); ++i) {
        if (result.size() >= kMaxCovers) return;
        if (!std::ranges::any_of(phases[i].groupIds,
                                 [pivot](GId g) { return g == pivot; })) continue;

        current.push_back(phases[i]);
        std::vector<GId> added;
        for (GId g : phases[i].groupIds)
            if (g < (int)covered.size() && !covered[g] &&
                std::ranges::binary_search(sortedReq, g))
            { covered[g] = true; added.push_back(g); }

        enumCoversRec(phases, sortedReq, i + 1, redundantBudget, current,
                      covered, uncovered - (int)added.size(), result);

        current.pop_back();
        for (GId g : added) covered[g] = false;
    }
}

}  // namespace

std::vector<std::vector<GPhase>> generateCovers(
    const std::vector<GPhase>& validPhases, const GConfig& cfg) {
    if (validPhases.empty()) return {};

    std::vector<GId> sortedReq;
    for (const auto& g : cfg.groups)
        if (g.kind != GKind::Crosswalk) sortedReq.push_back(g.id);
    std::ranges::sort(sortedReq);
    if (sortedReq.empty()) return {};

    GId maxGId = sortedReq.back();
    std::vector<bool> covered(maxGId + 1, false);
    std::vector<std::vector<GPhase>> result;
    std::vector<GPhase> current;

    enumCoversRec(validPhases, sortedReq, 0, kRedundantPhases, current, covered,
                  static_cast<int>(sortedReq.size()), result);
    return result;
}
