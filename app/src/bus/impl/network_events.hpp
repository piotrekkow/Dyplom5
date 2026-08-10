#pragma once

#include <road_network/util/ids.hpp>

template <typename ChildId, typename ParentId = void>
struct Created {
    ChildId id;
    ParentId at;
};

template <typename ChildId, typename ParentId = void>
struct Updated {
    ChildId id;
    ParentId at;
};

template <typename ChildId, typename ParentId = void>
struct Removed {
    ChildId id;
    ParentId at;
};

template <typename ChildId>
struct Created<ChildId, void> {
    ChildId id;
};

template <typename ChildId>
struct Updated<ChildId, void> {
    ChildId id;
};

template <typename ChildId>
struct Removed<ChildId, void> {
    ChildId id;
};

template <class Scope, class DirtyId>
struct Set {
    DirtyId at;
};

template <class Scope, class DirtyId>
struct Cleared {
    DirtyId at;
};