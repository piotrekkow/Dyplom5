#pragma once
#include <common/id.hpp>

// graph ids
struct NodeTag {};
struct EntryTag {};
struct MovementTag {};
struct ExitTag {};
struct CrosswalkTag {};
struct CrosswalkSeriesTag {};
struct ClusterTag {};

using NodeId = Id<NodeTag>;
using EntryId = Id<EntryTag>;
using MovementId = Id<MovementTag>;
using ExitId = Id<ExitTag>;
using CrosswalkId = Id<CrosswalkTag>;
using CrosswalkSeriesId = Id<CrosswalkSeriesTag>;
using ClusterId = Id<ClusterTag>;

// signal ids
struct StageTag {};
struct StageTransitionTag {};
struct TimingTag {};
struct SignalGroupTag {};

using StageId = Id<StageTag>;
using StageTransitionId = Id<StageTransitionTag>;
using TimingId = Id<TimingTag>;
using SignalGroupId = Id<SignalGroupTag>;

// variants
#include <variant>
using StreamId = std::variant<MovementId, CrosswalkId>;