#pragma once

#include <cassert>
#include <concepts>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "concepts.hpp"

// A sparse square matrix keyed by an id-like type on both axes. The diagonal is
// always empty: find(k, k) returns nullptr (a key never relates to itself).
template <typename Id, typename Item>
    requires Hashable<Id> && std::equality_comparable<Id>
class Matrix {
   public:
    using Row = std::unordered_map<Id, Item>;

    [[nodiscard]] const Item* find(Id row, Id col) const {
        if (row == col) return nullptr;
        auto rowIt = rows_.find(row);
        if (rowIt == rows_.end()) return nullptr;
        auto colIt = rowIt->second.find(col);
        if (colIt == rowIt->second.end()) return nullptr;
        return &colIt->second;
    }

    void set(Id row, Id col, Item valueRowCol, Item valueColRow) {
        rows_[row][col] = std::move(valueRowCol);
        rows_[col][row] = std::move(valueColRow);
    }

    void reset(Id row, Id col) {
        auto has = [&](Id a, Id b) {
            auto it = rows_.find(a);
            return it != rows_.end() && it->second.contains(b);
        };
        const bool fwd = has(row, col);
        const bool rev = has(col, row);
        assert(fwd == rev && "matrix must be symmetric");

        if (auto it = rows_.find(row); it != rows_.end()) it->second.erase(col);
        if (auto it = rows_.find(col); it != rows_.end()) it->second.erase(row);
    }

    [[nodiscard]] Matrix combined(const Matrix& overrides) const {
        Matrix result = *this;
        for (const auto& [row, cols] : overrides.rows_)
            for (const auto& [col, v] : cols) result.put(row, col, v);
        return result;
    }

    template <typename F>
    void forEach(F&& f) const {
        for (const auto& [row, cols] : rows_)
            for (const auto& [col, v] : cols) f(row, col, v);
    }

    // Visits each unordered pair once as f(row, col, valueRowCol, valueColRow).
    // Both directional values are always present by the symmetry invariant.
    template <typename F>
    void forEachPair(F&& f) const {
        std::unordered_set<Id> done;
        for (const auto& [row, cols] : rows_) {
            for (const auto& [col, v] : cols) {
                if (done.contains(col)) continue;  // emitted from col's row
                const Item* rev = find(col, row);
                assert(rev && "matrix must be symmetric");
                f(row, col, v, *rev);
            }
            done.insert(row);
        }
    }
    std::unordered_map<Id, Row>::size_type size() { return rows_.size(); }

   private:
    void put(Id row, Id col, Item v) { rows_[row][col] = std::move(v); }

    std::unordered_map<Id, Row> rows_;
};
