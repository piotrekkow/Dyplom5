#pragma once

#include <control/schema/group_params.hpp>
#include <regulations/types/turn_kind.hpp>
#include <unordered_map>

#include "ids.hpp"
#include "stream_matrix.hpp"

class GraphData;
class Regulations;
class Signal;
class DemandDerived;

namespace compute {
double classifyAngle(const GraphData& graph, EntryId entry, ExitId exit);
TurnKind turnKind(const GraphData& graph, const Regulations& regulations,
                  MovementId id);
std::unordered_map<SignalGroupId, double> groupMaxFlowRatios(
    const Signal& signal, const DemandDerived& demand, NodeId at);

std::unordered_map<SignalGroupId, control::GroupParams> groupParams(
    const Signal& signal, const Regulations& regulations, NodeId at);

// Max intergreen time for a directed (evac, appr) stream pair, or -1 if no
// constraint applies.
int resolveStreamIntergreen(const StreamMatrix<int>& intergreens,
                            const StreamMatrix<bool>& permitted, StreamId evac,
                            StreamId appr, bool evacProtected,
                            bool apprProtected);
}  // namespace compute