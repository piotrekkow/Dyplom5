#pragma once

#include <cassert>
#include <optional>
#include <road_network/util/id.hpp>
#include <type_traits>
#include <unordered_set>
#include <utility>

template <typename Tag, typename T>
class SlotMap {
   public:
    using IdType = Id<Tag>;

    template <bool IsConst>
    struct IteratorBase {
        using MapPtr = std::conditional_t<IsConst, const SlotMap*, SlotMap*>;
        using ValueRef = std::conditional_t<IsConst, const T&, T&>;

        struct Entry {
            IdType id;
            ValueRef value;
        };

        using difference_type = std::ptrdiff_t;
        using value_type = std::pair<IdType, T>;
        using reference = Entry;
        using iterator_category = std::forward_iterator_tag;

        struct ArrowProxy {
            reference ref;
            const reference* operator->() const { return &ref; }
        };

        MapPtr map_;
        uint32_t index_;

        IteratorBase(MapPtr map, uint32_t index) : map_(map), index_(index) {
            skip_dead();
        }

        reference operator*() const {
            return {IdType{index_, map_->generations_[index_]},
                    *map_->data_[index_]};
        }

        ArrowProxy operator->() const { return {**this}; }

        IteratorBase& operator++() {
            ++index_;
            skip_dead();
            return *this;
        }

        IteratorBase operator++(int) {
            auto tmp = *this;
            ++(*this);
            return tmp;
        }

        bool operator==(const IteratorBase&) const = default;

       private:
        void skip_dead() {
            while (index_ < map_->data_.size() &&
                   !map_->data_[index_].has_value()) {
                ++index_;
            }
        }
    };

    using Iterator = IteratorBase<false>;
    using ConstIterator = IteratorBase<true>;

    Iterator begin() { return {this, 0}; }
    Iterator end() { return {this, static_cast<uint32_t>(data_.size())}; }
    ConstIterator begin() const { return {this, 0}; }
    ConstIterator end() const {
        return {this, static_cast<uint32_t>(data_.size())};
    }

    IdType insert(T value) {
        uint32_t index;
        if (!free_.empty()) {
            index = free_.back();
            free_.pop_back();
            try {
                data_[index].emplace(std::move(value));
            } catch (...) {
                free_.push_back(index);
                throw;
            }
        } else {
            index = static_cast<uint32_t>(generations_.size());
            generations_.push_back(0);
            try {
                data_.emplace_back(std::move(value));
            } catch (...) {
                generations_.pop_back();  // restore invariant
                throw;
            }
        }
        return IdType{index, generations_[index]};
    }

    template <typename... Args>
    IdType emplace(Args&&... args) {
        uint32_t index;
        if (!free_.empty()) {
            index = free_.back();
            free_.pop_back();
            try {
                data_[index].emplace(std::forward<Args>(args)...);
            } catch (...) {
                free_.push_back(index);
                throw;
            }
            return IdType{index, generations_[index]};
        } else {
            index = static_cast<uint32_t>(generations_.size());
            generations_.push_back(0);
            try {
                data_.emplace_back(std::in_place, std::forward<Args>(args)...);
            } catch (...) {
                generations_.pop_back();  // restore invariant
                throw;
            }
        }
        return IdType{index, generations_[index]};
    }

    void erase(IdType id) { release(vacate(id)); }

    bool alive(IdType id) const {
        uint32_t index = id.index();
        if (index >= generations_.size()) return false;
        return generations_[index] == id.generation() &&
               data_[index].has_value();
    }

    struct VacatedSlot {
        IdType id;
        T value;

        VacatedSlot(IdType id_, T&& value_)
            : id(id_), value(std::move(value_)) {}

        VacatedSlot(VacatedSlot&&) = default;
        VacatedSlot& operator=(VacatedSlot&&) = default;
        VacatedSlot(const VacatedSlot&) = delete;
        VacatedSlot& operator=(const VacatedSlot&) = delete;
    };

    // empties, does NOT bump generation, slot is reserved for reinsert, returns
    // vacatedslot used for release/reinsertion
    VacatedSlot vacate(IdType id) {
        uint32_t index = id.index();
        assert(index < data_.size() && "tried to vacate invalid id");
        assert(generations_[index] == id.generation() &&
               "vacate generation mismatch");
        assert(data_[index].has_value() && "already vacated");
        static_assert(std::is_nothrow_destructible_v<VacatedSlot>);

        VacatedSlot vs{id, std::move(*data_[index])};
        data_[index].reset();
        vacated_.insert(index);
        return vs;
    }

    // frees id for reuse
    void release(VacatedSlot slot) {
        uint32_t index = slot.id.index();
        assert(index < data_.size() && "tried to release invalid id");
        assert(!data_[index].has_value() &&
               "ids released from slot map must be vacated first");
        assert(generations_[index] == slot.id.generation() &&
               "release generation mismatch");
        assert(vacated_.contains(index) &&
               "release on slot not in vacated state");

        vacated_.erase(index);
        ++generations_[index];
        free_.push_back(index);
    }

    // Because copy is deleted, any lambda capturing a VacatedSlot for the undo
    // closure must use init-capture by move
    void reinsert(VacatedSlot slot) {
        uint32_t index = slot.id.index();
        assert(index < data_.size() && "tried to reinsert on invalid id");
        assert(!data_[index].has_value() && "reinsert on live id");
        assert(generations_[index] == slot.id.generation() &&
               "reinsert generation mismatch");
        assert(vacated_.contains(index) &&
               "reinsert on slot not in vacated state");
        vacated_.erase(index);
        data_[index].emplace(std::move(slot.value));
    }

    T* get(IdType id) { return alive(id) ? &*data_[id.index()] : nullptr; }
    const T* get(IdType id) const {
        return alive(id) ? &*data_[id.index()] : nullptr;
    }

    size_t size() const {
        return data_.size() - free_.size() - vacated_.size();
    }

   private:
    std::vector<uint32_t> free_;
    std::vector<uint32_t> generations_;
    std::vector<std::optional<T>> data_;
    std::unordered_set<uint32_t> vacated_;
};