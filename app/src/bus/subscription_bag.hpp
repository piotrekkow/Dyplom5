#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "impl/event_bus.hpp"

class SubscriptionBag {
    struct Holder {
        virtual ~Holder() = default;
    };
    template <class E>
    struct TypedHolder : Holder {
        typename EventBus<E>::Subscription sub;
        explicit TypedHolder(typename EventBus<E>::Subscription s)
            : sub(std::move(s)) {}
    };

   public:
    template <class E, class F>
    void add(EventBus<E>& bus, F&& handler) {
        holders_.push_back(std::make_unique<TypedHolder<E>>(
            bus.subscribe(std::forward<F>(handler))));
    }

   private:
    std::vector<std::unique_ptr<Holder>> holders_;
};