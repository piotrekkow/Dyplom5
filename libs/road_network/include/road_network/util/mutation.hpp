#pragma once

#include <functional>

template <typename Id>
struct Mut {  // mutation
    Id id;
    std::move_only_function<void()> undo;
};