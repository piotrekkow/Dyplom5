#pragma once
#include <algorithm>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

template <typename Event>
class EventBus {
    using Handler = std::function<void(const Event&)>;
    using Entry   = std::pair<uint64_t, Handler>;

   public:
    EventBus() = default;
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;
    EventBus(EventBus&&) = delete;
    EventBus& operator=(EventBus&&) = delete;

    using SubId = uint64_t;

    class Subscription {
       public:
        Subscription(EventBus<Event>& bus, SubId id) : bus_(&bus), id_(id) {}
        ~Subscription() {
            if (bus_) bus_->unsubscribe(id_);
        }

        Subscription(const Subscription&) = delete;
        Subscription& operator=(const Subscription&) = delete;

        Subscription(Subscription&& other) noexcept
            : bus_(other.bus_), id_(other.id_) {
            other.bus_ = nullptr;
        }

        Subscription& operator=(Subscription&& other) noexcept {
            if (this != &other) {
                if (bus_) bus_->unsubscribe(id_);
                bus_ = other.bus_;
                id_ = other.id_;
                other.bus_ = nullptr;
            }
            return *this;
        }

       private:
        EventBus<Event>* bus_;
        SubId id_;
    };

    Subscription subscribe(Handler h) {
        SubId id = nextId_++;
        if (depth_ == 0)
            handlers_.emplace_back(id, std::move(h));
        else
            pendingInserts_.emplace_back(id, std::move(h));
        return Subscription{*this, id};
    }

    void publish(const Event& e) {
        ++depth_;
        for (const auto& [id, h] : handlers_) h(e);
        --depth_;
        if (depth_ == 0) flush();
    }

    void unsubscribe(SubId id) {
        if (depth_ > 0)
            pendingErases_.push_back(id);
        else
            eraseFromHandlers(id);
    }

   private:
    void eraseFromHandlers(SubId id) {
        auto it = std::ranges::find_if(handlers_,
            [id](const Entry& e) { return e.first == id; });
        if (it != handlers_.end()) {
            *it = std::move(handlers_.back());
            handlers_.pop_back();
        }
    }

    void flush() {
        for (SubId id : pendingErases_) eraseFromHandlers(id);
        for (auto& [id, h] : pendingInserts_)
            if (!std::ranges::contains(pendingErases_, id))
                handlers_.emplace_back(id, std::move(h));
        pendingErases_.clear();
        pendingInserts_.clear();
    }

    std::vector<Entry> handlers_;
    std::vector<Entry> pendingInserts_;
    std::vector<SubId> pendingErases_;
    SubId nextId_ = 0;
    int depth_ = 0;
};
