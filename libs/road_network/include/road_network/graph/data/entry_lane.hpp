#pragma once
#include <optional>

#include "road_network/util/ids.hpp"

class EntryLane {
   public:
    EntryLane(EntryId at) : entry_(at) {}

    [[nodiscard]] EntryId at() const { return entry_; }

    [[nodiscard]] float width() const { return width_; }
    [[nodiscard]] float stopLineOffset() const { return stopLineOffset_; }
    [[nodiscard]] std::optional<float> storageLength() const {
        return storageLength_;
    }

    void setWidth(float width) { width_ = width > 0.f ? width : width_; }

    void setStopLineOffset(float offset) {
        stopLineOffset_ = offset > 0.f ? offset : stopLineOffset_;
    }

    void setStorageLength(std::optional<float> length) {
        if (length) {
            auto val = length.value();
            storageLength_ = val > 0.f ? val : storageLength_;
        } else
            storageLength_ = std::nullopt;
    }

   private:
    EntryId entry_;
    float width_ = 3.5f;
    float stopLineOffset_ = 0.f;
    std::optional<float> storageLength_ = std::nullopt;
};