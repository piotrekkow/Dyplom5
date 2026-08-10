#pragma once

#include <concepts>
#include <cstddef>
#include <functional>

template <typename T>
concept Hashable = requires(const T& v) {
    { std::hash<T>{}(v) } -> std::convertible_to<std::size_t>;
};

template <typename T>
concept ComparableKey =
    Hashable<T> && std::equality_comparable<T> && std::copyable<T>;