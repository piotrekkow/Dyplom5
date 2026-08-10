#pragma once

#include <QGraphicsScene>
#include <common/concepts.hpp>
#include <concepts>
#include <functional>
#include <unordered_map>

class QGraphicsItem;

template <typename Id, typename Item>
    requires Hashable<Id> && std::equality_comparable<Id>
class ItemRegistry {
   public:
    ItemRegistry(QGraphicsScene& scene,
                 std::function<void(QGraphicsItem*)> onAdded)
        : scene_{scene}, onAdded_{std::move(onAdded)} {}

    template <class... Args>
    Item* insert(Id id, Args&&... args) {
        auto* item = new Item(id, std::forward<Args>(args)...);
        scene_.addItem(item);
        map_.emplace(id, item);
        if (onAdded_) onAdded_(item);
        return item;
    }

    void remove(Id id) {
        auto it = map_.find(id);
        if (it == map_.end()) return;  // no-op for unknown ids
        scene_.removeItem(it->second);
        delete it->second;
        map_.erase(it);
    }

    [[nodiscard]] Item* find(Id id) const {
        auto it = map_.find(id);
        return it != map_.end() ? it->second : nullptr;
    }

   private:
    QGraphicsScene& scene_;
    std::function<void(QGraphicsItem*)> onAdded_;
    std::unordered_map<Id, Item*> map_;
};