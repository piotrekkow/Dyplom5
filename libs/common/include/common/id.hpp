#pragma once

#include <cstdint>
#include <functional>

template <typename Tag>
class Id {
   public:
    [[nodiscard]] constexpr uint32_t index() const {
        return static_cast<uint32_t>(value_ & 0xFFFFFFFFull);
    }
    [[nodiscard]] constexpr uint32_t generation() const {
        return static_cast<uint32_t>(value_ >> 32);
    }

    friend constexpr bool operator==(const Id&, const Id&) = default;
    friend constexpr bool operator<(const Id& a, const Id& b) {
        return a.value_ < b.value_;
    }
    friend struct std::hash<Id<Tag>>;

   private:
    template <typename, typename>
    friend class SlotMap;
    constexpr Id(uint32_t index, uint32_t generation)
        : value_((static_cast<uint64_t>(generation) << 32) | index) {}
    uint64_t value_;
};

template <typename Tag>
struct std::hash<Id<Tag>> {
    size_t operator()(const Id<Tag>& id) const noexcept {
        return std::hash<uint64_t>()(id.value_);
    }
};