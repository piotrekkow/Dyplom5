#pragma once

#include <functional>

template <typename ChildId, typename ParentId>
struct EMut {  // entity mutation
    ChildId id;
    ParentId at;
    std::move_only_function<void()> undo;
};