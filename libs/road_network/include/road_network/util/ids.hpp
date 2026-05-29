#pragma once
#include <road_network/util/id.hpp>
#include <variant>

// graph id
struct NodeTag {};
struct EdgeTag {};
struct EntryTag {};
struct MovementTag {};
struct ExitTag {};
struct EntryLaneTag {};
struct ExitLaneTag {};
struct CrosswalkTag {};
struct CrosswalkSeriesTag {};
struct ClusterTag {};

using NodeId = Id<NodeTag>;
using EdgeId = Id<EdgeTag>;
using EntryId = Id<EntryTag>;
using MovementId = Id<MovementTag>;
using ExitId = Id<ExitTag>;
using EntryLaneId = Id<EntryLaneTag>;
using ExitLaneId = Id<ExitLaneTag>;
using CrosswalkId = Id<CrosswalkTag>;
using CrosswalkSeriesId = Id<CrosswalkSeriesTag>;
using ClusterId = Id<ClusterTag>;

// signal ids
struct MovementSignalGroupTag {};
struct CrosswalkSignalGroupTag {};
struct PhaseTag {};
struct PhaseTransitionTag {};
struct TimingTag {};
struct SignalGroupTag {};

using MovementSignalGroupId = Id<MovementSignalGroupTag>;
using CrosswalkSignalGroupId = Id<CrosswalkSignalGroupTag>;
using PhaseId = Id<PhaseTag>;
using PhaseTransitionId = Id<PhaseTransitionTag>;
using TimingId = Id<TimingTag>;
using SignalGroupId = Id<SignalGroupTag>;

// variants
#include <variant>
using StreamId = std::variant<MovementId, CrosswalkId>;
using AdapterId = std::variant<EntryId, ExitId>;