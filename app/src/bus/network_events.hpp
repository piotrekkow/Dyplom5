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
struct EntryMoved {
    EntryId id;
    NodeId at;
};
struct EntrySpeedLimitChanged {
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
struct ExitMoved {
    ExitId id;
    NodeId at;
};
struct ExitSpeedLimitChanged {
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
struct CrosswalkSeriesWalkSpeedChanged {
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

// published after the deferred demand solve — laneSaturations of the entry's
// lanes may have changed (mirror of NodeRederived)
struct DemandRederived {
    EntryId id;
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

struct IntersectionCreated {
    IntersectionId id;
};
struct IntersectionRemoved {
    IntersectionId id;
};
struct IntersectionNodesChanged {
    IntersectionId id;
};

struct SignalConfigCreated {
    SignalConfigId id;
    IntersectionId at;
};
struct SignalConfigRemoved {
    SignalConfigId id;
    IntersectionId at;
};
struct SignalConfigRenamed {
    SignalConfigId id;
    IntersectionId at;
};

struct SignalGroupCreated {
    SignalGroupId id;
    SignalConfigId config;
    IntersectionId at;
};
struct SignalGroupRemoved {
    SignalGroupId id;
    SignalConfigId config;
    IntersectionId at;
};
struct SignalGroupChanged {
    SignalGroupId id;
    SignalConfigId config;
    IntersectionId at;
};

// control domain
struct TimingCreated {
    TimingId id;
    SignalConfigId config;
};
struct TimingRemoved {
    TimingId id;
    SignalConfigId config;
};

struct SequenceChanged {
    TimingId timingId;
    SignalGroupId sgId;
    SignalConfigId config;
};

// published after the deferred control validation pass (mirror of
// NodeRederived) — re-read Control::derived() for current flags
struct TimingValidityChanged {
    TimingId id;
    SignalConfigId config;
};
struct ConfigCompletenessChanged {
    SignalConfigId id;
    IntersectionId at;
};