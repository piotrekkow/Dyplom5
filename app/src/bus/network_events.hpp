#pragma once

#include <road_network/util/ids.hpp>

struct NodeCreated {
    NodeId id;
};
struct NodeRemoved {
    NodeId id;
};

struct NodeRederived {
    NodeId id;
};

struct EntryCreated {
    EntryId id;
    NodeId at;
};
struct EntryRemoved {
    EntryId id;
    NodeId at;
};

struct ExitCreated {
    ExitId id;
    NodeId at;
};
struct ExitRemoved {
    ExitId id;
    NodeId at;
};

struct CrosswalkSeriesCreated {
    CrosswalkSeriesId id;
    NodeId at;
};
struct CrosswalkSeriesUpdated {
    CrosswalkSeriesId id;
    NodeId at;
};
struct CrosswalkSeriesRemoved {
    CrosswalkSeriesId id;
    NodeId at;
};

struct CrosswalkCreated {
    CrosswalkId id;
    NodeId at;
};
struct CrosswalkUpdated {
    CrosswalkId id;
    NodeId at;
};
struct CrosswalkRemoved {
    CrosswalkId id;
    NodeId at;
};

struct MovementCreated {
    MovementId id;
    NodeId at;
};
struct MovementUpdated {
    MovementId id;
    NodeId at;
};
struct MovementRemoved {
    MovementId id;
    NodeId at;
};

struct EntryLanesChanged {
    EntryId id;
    NodeId at;
};

struct ExitLanesChanged {
    ExitId id;
    NodeId at;
};

struct ClusterCreated {
    ClusterId id;
    NodeId at;
};
struct ClusterRemoved {
    ClusterId id;
    NodeId at;
};

struct FlowChanged {
    MovementId at;
};
struct FlowErased {
    MovementId at;
};

struct SaturationOverrideChanged {
    MovementId at;
};
struct SaturationOverrideErased {
    MovementId at;
};

struct StreamPermittedOverrideChanged {
    NodeId at;
};
struct StreamPermittedOverrideErased {
    NodeId at;
};

struct StreamIntergreenOverrideChanged {
    NodeId at;
};
struct StreamIntergreenOverrideErased {
    NodeId at;
};

struct SignalGroupCreated {
    SignalGroupId id;
    NodeId at;
};
struct SignalGroupRemoved {
    SignalGroupId id;
    NodeId at;
};
struct SignalGroupChanged {
    SignalGroupId id;
    NodeId at;
};

struct TimingCreated {
    TimingId id;
    NodeId at;
};
struct TimingRemoved {
    TimingId id;
    NodeId at;
};

struct SequenceChanged {
    TimingId timingId;
    SignalGroupId sgId;
    NodeId at;
};