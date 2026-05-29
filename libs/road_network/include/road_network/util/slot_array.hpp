#pragma once

#include <optional>
#include <road_network/util/id.hpp>

template <typename Tag, typename T>
class SlotArray {
   public:
    void set(Id<Tag> id, T value) {
        if (id.index() >= data_.size()) data_.resize(id.index() + 1);
        data_[id.index()] = std::move(value);
    }

    // important: when using ensure id alive via check through master slotmap
    const T* get(Id<Tag> id) const {
        if (id.index() >= data_.size()) return nullptr;
        return data_[id.index()].has_value() ? &*data_[id.index()] : nullptr;
    }

    T* get(Id<Tag> id) {
        if (id.index() >= data_.size()) return nullptr;
        return data_[id.index()].has_value() ? &*data_[id.index()] : nullptr;
    }

    void erase(Id<Tag> id) {
        if (id.index() < data_.size()) data_[id.index()].reset();
    }

   private:
    std::vector<std::optional<T>> data_;
};