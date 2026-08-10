#pragma once

#include <common/id.hpp>
#include <common/slot_array.hpp>
#include <common/slot_map.hpp>
#include <concepts>
#include <unordered_set>

#include "entity_mutation.hpp"
#include "mutation.hpp"

template <typename T>
concept ParamEntity = std::copyable<typename T::Params> && requires(T t) {
    { t.params() } -> std::same_as<typename T::Params&>;
};

template <typename... ParentIds>
class MutationBase {
   public:
    template <typename Self, typename P>
        requires(std::same_as<P, ParentIds> || ...)
    void markDirty(this Self&& self, P id) {
        std::get<std::unordered_set<P>>(self.dirty_).insert(id);
    }

   protected:
    template <typename P, typename Self>
    std::unordered_set<P>& dirty(this Self&& self) {
        return std::get<std::unordered_set<P>>(self.dirty_);
    }

    template <typename Self, typename Tag, typename T, typename... Args>
        requires requires(Self s, Id<Tag> id) {
            s.link(id);
            s.parentOf(id);
        }
    Id<Tag> createChild(this Self&& self, SlotMap<Tag, T>& map,
                        Args&&... args) {
        Id<Tag> id = map.emplace(std::forward<Args>(args)...);
        self.link(id);
        self.markDirty(self.parentOf(id));
        return id;
    }

    template <typename Self, typename Tag, typename T>
        requires ParamEntity<T> &&
                 requires(Self s, Id<Tag> id) { s.parentOf(id); }
    auto setParams(this Self&& self, SlotMap<Tag, T>& map, Id<Tag> id,
                   typename T::Params p) {
        assert(map.alive(id));
        using ParentId = decltype(self.parentOf(id));
        const ParentId n = self.parentOf(id);
        typename T::Params old = map.get(id)->params();
        map.get(id)->params() = std::move(p);
        self.markDirty(n);
        return EMut<Id<Tag>, ParentId>{
            .id = id,
            .at = n,
            .undo = [domain = &self, &map, id, n,
                     old = std::move(old)]() mutable {
                map.get(id)->params() = std::move(old);
                domain->markDirty(n);
            }};
    }

    template <typename Self, typename Tag, typename T>
        requires requires(Self s, Id<Tag> id) {
            s.unlink(id);
            s.parentOf(id);
        }
    typename SlotMap<Tag, T>::VacatedSlot detach(this Self&& self,
                                                 SlotMap<Tag, T>& map,
                                                 Id<Tag> id) {
        assert(map.alive(id));
        auto parent = self.parentOf(id);
        self.unlink(id);
        auto vs = map.vacate(id);
        self.markDirty(parent);
        return vs;
    }

    template <typename Self, typename Tag, typename T>
        requires requires(Self s, Id<Tag> id) {
            s.link(id);
            s.parentOf(id);
        }
    void attach(this Self&& self, SlotMap<Tag, T>& map,
                typename SlotMap<Tag, T>::VacatedSlot slot) {
        Id<Tag> id = slot.id;
        map.reinsert(std::move(slot));
        self.link(id);
        self.markDirty(self.parentOf(id));
    }

    template <typename Self, typename Tag, typename T>
        requires requires(Self s, Id<Tag> id) { s.markDirty(id); }
    auto setSlot(this Self&& self, SlotArray<Tag, T>& arr, Id<Tag> id,
                 T value) {
        std::optional<T> old;
        if (T* curr = arr.get(id)) old = std::move(*curr);
        arr.set(id, std::move(value));
        self.markDirty(id);
        return Mut<Id<Tag>>{
            .id = id,
            .undo = [domain = &self, &arr, id, old = std::move(old)] mutable {
                old ? arr.set(id, std::move(*old)) : arr.erase(id);
                domain->markDirty(id);
            }};
    }

    // for when there is no self.parentOf() accessible
    template <typename Self, typename Tag, typename T, typename Parent>
        requires requires(Self s, Parent at) { s.markDirty(at); } &&
                 (!requires(Self s, Id<Tag> id) { s.parentOf(id); })
    auto setSlotAt(this Self&& self, SlotArray<Tag, T>& arr, Id<Tag> id,
                   Parent at, T value) {
        std::optional<T> old;
        if (T* curr = arr.get(id)) old = std::move(*curr);
        arr.set(id, std::move(value));
        self.markDirty(at);  // node, not id
        return Mut<Id<Tag>>{.id = id,
                            .undo = [domain = &self, &arr, id, at,
                                     old = std::move(old)]() mutable {
                                old ? arr.set(id, std::move(*old))
                                    : arr.erase(id);
                                domain->markDirty(at);
                            }};
    }

    template <typename Self, typename Tag, typename T, typename Parent>
        requires requires(Self s, Parent at) { s.markDirty(at); } &&
                 (!requires(Self s, Id<Tag> id) { s.parentOf(id); })
    auto clearSlotAt(this Self&& self, SlotArray<Tag, T>& arr, Id<Tag> id,
                     Parent at) {
        std::optional<T> old;
        if (T* curr = arr.get(id)) old = std::move(*curr);
        arr.erase(id);
        self.markDirty(at);
        return Mut<Id<Tag>>{.id = id,
                            .undo = [domain = &self, &arr, id, at,
                                     old = std::move(old)]() mutable {
                                if (old) arr.set(id, std::move(*old));
                                domain->markDirty(at);
                            }};
    }

    std::tuple<std::unordered_set<ParentIds>...> dirty_;
};